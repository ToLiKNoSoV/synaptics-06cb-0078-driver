/*
 * libfprint driver for Synaptics 06cb:0078 fingerprint sensor
 * Based on reverse engineering work and Python implementation
 *
 * Copyright (C) 2026 Anatoliy Nosov <toliknosov1994@eyandex.ru>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#define FP_COMPONENT "synaptics_0078"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <pwd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <fpi-print.h>
#include <fpi-log.h>
#include <fpi-device.h>
#include <fpi-ssm.h>
#include <fpi-usb-transfer.h>
#include <fp-image.h>
#include <fpi-image.h>
#include <fpi-image-device.h>
#include <fprint.h>

#include "synaptics_0078_data.c"  /* Подключаем данные */

/* USB IDs */
#define SYNAPTICS_0078_VID 0x06cb
#define SYNAPTICS_0078_PID 0x0078

/* Endpoints - согласно Python скрипту */
#define EP_DATA_OUT        0x01    /* OUT endpoint для команд */
#define EP_DATA_IN         0x81    /* IN endpoint для ВСЕХ данных (и статуса, и изображения) */
#define EP_INTR            0x83    /* Interrupt endpoint для событий */

/* Image dimensions */
#define IMG_WIDTH          144
#define IMG_HEIGHT         56
#define IMG_SIZE           (IMG_WIDTH * IMG_HEIGHT)  /* 8064 */
#define IMG_PACKET_SIZE    64  /* Стандартный размер пакета для USB FULL-SPEED */
#define IMG_TOTAL_SIZE     8082 /* Полный размер изображения с заголовком */

/* Known interrupt patterns */
static const guint8 INTR_FINGER_ON[] =  {0x02, 0x00, 0x40, 0x10, 0x00};
static const guint8 INTR_FINGER_OK[] =  {0x03, 0x40, 0x01, 0x00, 0x00};
static const guint8 INTR_FINGER_BAD[] = {0x03, 0x50, 0x05, 0x00, 0x00};
static const guint8 INTR_READY3[] =     {0x03, 0x43, 0x04, 0x00, 0x41};
static const guint8 INTR_IMAGE_READY[] = {0x03, 0x42, 0x04, 0x00, 0x40};

/* Command buffers */
static const guint8 CMD_CAPTURE[] = {0x51, 0x00, 0x20, 0x00, 0x00};
static const guint8 CMD_INIT_01[] = {0x01};
static const guint8 CMD_INIT_19[] = {0x19};
static const guint8 CMD_INIT_0860[] = {0x08, 0x60, 0x20, 0x00, 0x80, 0x0b, 0x00, 0x00, 0x00, 0x04};
static const guint8 CMD_INIT_0780[] = {0x07, 0x80, 0x20, 0x00, 0x80, 0x04};
static const guint8 CMD_INIT_75[] = {0x75};

/* Вспомогательные функции для обработки изображений */
FpImage *fpi_image_binarize(FpImage *image);
FpImage *fpi_image_scale_to_size(FpImage *image, int new_width, int new_height);
gboolean fpi_image_check_quality(FpImage *image, GError **error);

/* Status codes */
#define STATUS_IMAGE_READY      0x0604
#define STATUS_NO_FINGER        0x0100
#define STATUS_IMAGE_GOOD       0x0000
#define STATUS_IMAGE_BAD        0x0700

/* States for initialization */
enum {
    INIT_STATE_START,
    INIT_STATE_SEND_01,
    INIT_STATE_READ_38,
    INIT_STATE_SEND_19,
    INIT_STATE_READ_68,
    INIT_STATE_SEND_0860,
    INIT_STATE_READ_2,
    INIT_STATE_SEND_0780,
    INIT_STATE_READ_6,
    INIT_STATE_SEND_75,
    INIT_STATE_READ_10,
    INIT_STATE_SEND_01_AGAIN,
    INIT_STATE_READ_38_AGAIN,
    INIT_STATE_SEND_19_AGAIN,
    INIT_STATE_READ_68_AGAIN,
    INIT_STATE_SEND_75_AGAIN,
    INIT_STATE_READ_10_AGAIN,
    INIT_STATE_SEND_LONG1,
    INIT_STATE_READ_2_LONG1,
    INIT_STATE_SEND_LONG2,
    INIT_STATE_READ_2154,
    INIT_STATE_SEND_LONG3,
    INIT_STATE_READ_2154_AGAIN,
    INIT_STATE_COMPLETE,
    INIT_NUM_STATES
};

/* States for capture */
enum {
    CAPTURE_START,
    CAPTURE_SEND_CAPTURE_CMD,
    CAPTURE_READ_STATUS,
    CAPTURE_WAIT_FINGER_ON,
    CAPTURE_WAIT_FINGER_OK,
    CAPTURE_SEND_CAPTURE_AGAIN,
    CAPTURE_READ_IMAGE_HEADER,
    CAPTURE_READ_IMAGE_DATA,
    CAPTURE_PROCESS_IMAGE,
    CAPTURE_NUM_STATES
};

typedef enum {
    MODE_NONE,
    MODE_CAPTURE,
    MODE_ENROLL,
    MODE_VERIFY
} DeviceMode;

typedef struct _FpiDeviceSynaptics0078 FpiDeviceSynaptics0078;
typedef struct _FpiDeviceSynaptics0078Class FpiDeviceSynaptics0078Class;

struct _FpiDeviceSynaptics0078 {
    FpDevice            parent;
    gboolean            initialized;
    FpiSsm             *init_ssm;
    FpiSsm             *capture_ssm;
    FpiUsbTransfer     *interrupt_transfer;
    guint8             *image_buffer;
    gsize               image_buffer_len;
    gsize               image_bytes_received;
    guint16             image_status;
    gboolean            finger_detected;
    DeviceMode          mode;
    char               *username;
    char               *finger_name;
};

struct _FpiDeviceSynaptics0078Class {
    FpDeviceClass parent_class;
};

/* Прототипы для G_DEFINE_TYPE */
GType fpi_device_synaptics_0078_get_type(void);

// Макрос для приведения типа
#define FPI_DEVICE_SYNAPTICS_0078(dev) ((FpiDeviceSynaptics0078 *) (dev))

G_DEFINE_TYPE(FpiDeviceSynaptics0078, fpi_device_synaptics_0078, FP_TYPE_DEVICE)

/* ========== Вспомогательные функции ========== */

/* Преобразование имени пальца в enum FpFinger */
static FpFinger finger_name_to_enum(const char *name)
{
    if (!name) return FP_FINGER_UNKNOWN;

    if (g_strcmp0(name, "right-index-finger") == 0)
        return FP_FINGER_RIGHT_INDEX;
    if (g_strcmp0(name, "right-middle-finger") == 0)
        return FP_FINGER_RIGHT_MIDDLE;
    if (g_strcmp0(name, "right-ring-finger") == 0)
        return FP_FINGER_RIGHT_RING;
    if (g_strcmp0(name, "right-little-finger") == 0)
        return FP_FINGER_RIGHT_LITTLE;
    if (g_strcmp0(name, "left-index-finger") == 0)
        return FP_FINGER_LEFT_INDEX;
    if (g_strcmp0(name, "left-middle-finger") == 0)
        return FP_FINGER_LEFT_MIDDLE;
    if (g_strcmp0(name, "left-ring-finger") == 0)
        return FP_FINGER_LEFT_RING;
    if (g_strcmp0(name, "left-little-finger") == 0)
        return FP_FINGER_LEFT_LITTLE;
    if (g_strcmp0(name, "right-thumb") == 0)
        return FP_FINGER_RIGHT_THUMB;
    if (g_strcmp0(name, "left-thumb") == 0)
        return FP_FINGER_LEFT_THUMB;

    return FP_FINGER_UNKNOWN;
}

/* Forward declarations */
static void interrupt_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error);
static void write_cmd_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error);
static void read_response_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error);
static void read_image_chunk_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error);
static void init_ssm_run(FpiSsm *ssm, FpDevice *dev);
static void init_ssm_complete(FpiSsm *ssm, FpDevice *dev, GError *error);
static void capture_ssm_run(FpiSsm *ssm, FpDevice *dev);
static void capture_ssm_complete(FpiSsm *ssm, FpDevice *dev, GError *error);
static void start_interrupt_polling(FpiDeviceSynaptics0078 *self);
static void continue_image_read(FpiDeviceSynaptics0078 *self);
static void start_capture_sequence(FpiDeviceSynaptics0078 *self);
static void fpi_device_synaptics_0078_enroll(FpDevice *dev);
static void fpi_device_synaptics_0078_verify(FpDevice *dev);
static void fpi_device_synaptics_0078_cancel(FpDevice *dev);

/* ========== USB Callbacks ========== */

static void write_cmd_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
    FpiSsm *ssm = user_data;

    if (error) {
        fp_err("Write error: %s", error->message);
        fpi_ssm_mark_failed(ssm, error);
        return;
    }

    fp_dbg("Command sent, %zd bytes", transfer->actual_length);
    fpi_ssm_next_state(ssm);
}

static void read_response_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
    FpiSsm *ssm = user_data;

    if (error) {
        fp_err("Read error: %s", error->message);
        fpi_ssm_mark_failed(ssm, error);
        return;
    }

    fp_dbg("Response received: %zd bytes", transfer->actual_length);

    if (transfer->actual_length > 0) {
        fp_dbg("First bytes: %02x %02x %02x %02x...",
               transfer->buffer[0], transfer->buffer[1],
               transfer->buffer[2], transfer->buffer[3]);
    }

    fpi_ssm_next_state(ssm);
}

static void read_image_chunk_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);
    FpiSsm *ssm = self->capture_ssm;
    int current_state = fpi_ssm_get_cur_state(ssm);

    if (!ssm) {
        fp_err("No active SSM");
        return;
    }

    if (error) {
        fp_err("Image read error: %s", error->message);
        fpi_ssm_mark_failed(ssm, error);
        return;
    }

    fp_dbg("Image chunk received: %zd bytes, actual_length: %zd, state: %d",
           transfer->actual_length, transfer->actual_length, current_state);

    if (transfer->actual_length == 2 && current_state == CAPTURE_READ_STATUS) {
        guint16 status = (transfer->buffer[0] << 8) | transfer->buffer[1];
        fp_dbg("Got status response: 0x%04x", status);

        if (status == 0x0604) {
            fp_dbg("Image is being prepared, waiting for finger...");
            fpi_ssm_next_state(ssm);
            return;
        } else if (status == 0x0100) {
            fp_dbg("No finger detected");
            fpi_ssm_mark_failed(ssm,
                                fpi_device_error_new_msg(FP_DEVICE_ERROR_DATA_INVALID,
                                                         "No finger detected"));
            return;
        }
    }

    if (transfer->actual_length > 0 && current_state >= CAPTURE_READ_IMAGE_HEADER) {
        if (self->image_bytes_received + transfer->actual_length > self->image_buffer_len) {
            fp_err("Image buffer overflow: %zd + %zd > %zd",
                   self->image_bytes_received, transfer->actual_length, self->image_buffer_len);
            fpi_ssm_mark_failed(ssm,
                                fpi_device_error_new_msg(FP_DEVICE_ERROR_DATA_INVALID,
                                                         "Image buffer overflow"));
            return;
        }

        memcpy(self->image_buffer + self->image_bytes_received,
               transfer->buffer, transfer->actual_length);
        self->image_bytes_received += transfer->actual_length;

        fp_dbg("Total received: %zd/%zd", self->image_bytes_received, self->image_buffer_len);

        if (self->image_bytes_received >= 18 && current_state == CAPTURE_READ_IMAGE_HEADER) {
            guint16 status = (self->image_buffer[0] << 8) | self->image_buffer[1];
            fp_dbg("Header status: 0x%04x", status);

            for (int i = 0; i < self->image_bytes_received - 1; i++) {
                if (self->image_buffer[i] == 0x8C && self->image_buffer[i+1] == 0x1F) {
                    fp_dbg("Found image marker 0x8C1F at offset %d", i);
                    break;
                }
            }

            fpi_ssm_next_state(ssm);
            return;
        }

        if (self->image_bytes_received >= IMG_TOTAL_SIZE) {
            fp_dbg("Image complete");
            fpi_ssm_next_state(ssm);
            return;
        }
    }

    if (current_state >= CAPTURE_READ_IMAGE_DATA) {
        continue_image_read(self);
    }
}

static void continue_image_read(FpiDeviceSynaptics0078 *self)
{
    FpDevice *dev = FP_DEVICE(self);
    FpiUsbTransfer *transfer;

    fp_dbg("Continuing image read, need %zd more bytes",
           self->image_buffer_len - self->image_bytes_received);

    transfer = fpi_usb_transfer_new(dev);
    if (!transfer) {
        fp_err("Failed to create transfer");
        fpi_ssm_mark_failed(self->capture_ssm,
                            fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
        return;
    }

    fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, IMG_PACKET_SIZE);
    fpi_usb_transfer_submit(transfer, 5000,
                            fpi_device_get_cancellable(dev),
                            read_image_chunk_cb, self);
}

/* ========== Interrupt Handling ========== */

static void interrupt_cb(FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);

    if (!self) {
        fp_err("interrupt_cb: self is NULL");
        return;
    }

    FpiSsm *ssm = self->capture_ssm;
    int current_state = -1;

    if (ssm) {
        current_state = fpi_ssm_get_cur_state(ssm);
        fp_dbg("Current SSM state: %d", current_state);
    }

    if (error) {
        if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT)) {
            fp_dbg("Interrupt timeout");
        } else if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
            fp_dbg("Interrupt cancelled");
        } else {
            fp_err("Interrupt error: %s", error->message);
        }
        g_error_free(error);

        if (!g_cancellable_is_cancelled(fpi_device_get_cancellable(dev))) {
            FpiUsbTransfer *new_transfer = fpi_usb_transfer_new(dev);
            if (new_transfer) {
                fpi_usb_transfer_fill_interrupt(new_transfer, EP_INTR, 8);
                self->interrupt_transfer = new_transfer;
                fpi_usb_transfer_submit(new_transfer, 0,
                                        fpi_device_get_cancellable(dev),
                                        interrupt_cb, NULL);
            }
        }
        return;
    }

    if (transfer->actual_length > 0) {
        guint8 *data = transfer->buffer;
        fp_dbg("Interrupt received: %zd bytes, data: %02x %02x %02x %02x %02x...",
               transfer->actual_length, data[0], data[1], data[2], data[3], data[4]);

        if (memcmp(data, INTR_FINGER_ON, 5) == 0) {
            fp_dbg("✓ FINGER ON detected");
            self->finger_detected = TRUE;
            if (ssm && current_state == CAPTURE_WAIT_FINGER_ON) {
                // Небольшая задержка для стабилизации
                g_usleep(100000);
                fpi_ssm_next_state(ssm);
            }
        }
        else if (memcmp(data, INTR_FINGER_OK, 5) == 0 ||
            memcmp(data, INTR_READY3, 5) == 0 ||
            memcmp(data, INTR_IMAGE_READY, 5) == 0) {
            fp_dbg("✓ IMAGE READY detected");
        if (ssm && current_state == CAPTURE_WAIT_FINGER_OK) {
            g_usleep(200000);
            fpi_ssm_next_state(ssm);
        }
            }
            else if (memcmp(data, INTR_FINGER_BAD, 5) == 0) {
                fp_dbg("✗ FINGER BAD detected");
            }
    }

    FpiUsbTransfer *new_transfer = fpi_usb_transfer_new(dev);
    if (new_transfer) {
        fpi_usb_transfer_fill_interrupt(new_transfer, EP_INTR, 8);
        self->interrupt_transfer = new_transfer;
        fpi_usb_transfer_submit(new_transfer, 0,
                                fpi_device_get_cancellable(dev),
                                interrupt_cb, NULL);
    }
}

static void start_interrupt_polling(FpiDeviceSynaptics0078 *self)
{
    FpDevice *dev = FP_DEVICE(self);

    if (self->interrupt_transfer) {
        fp_dbg("Interrupt polling already active");
        return;
    }

    fp_dbg("Starting interrupt polling on EP 0x%02x", EP_INTR);

    FpiUsbTransfer *transfer = fpi_usb_transfer_new(dev);
    if (!transfer) {
        fp_err("Failed to create interrupt transfer");
        return;
    }

    fpi_usb_transfer_fill_interrupt(transfer, EP_INTR, 8);
    self->interrupt_transfer = transfer;
    fpi_usb_transfer_submit(transfer, 0,
                            fpi_device_get_cancellable(dev),
                            interrupt_cb, NULL);
}

/* ========== Initialization SSM ========== */

static void init_ssm_run(FpiSsm *ssm, FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);
    int current_state = fpi_ssm_get_cur_state(ssm);
    FpiUsbTransfer *transfer;

    fp_dbg("Init state %d", current_state);

    switch (current_state) {
        case INIT_STATE_START:
            fp_dbg("Starting init sequence");
            fpi_ssm_next_state(ssm);
            break;

        case INIT_STATE_SEND_01:
            fp_dbg("Sending 0x01");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_01));
            memcpy(transfer->buffer, CMD_INIT_01, sizeof(CMD_INIT_01));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_38:
            fp_dbg("Reading 38 bytes");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 38);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_19:
            fp_dbg("Sending 0x19");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_19));
            memcpy(transfer->buffer, CMD_INIT_19, sizeof(CMD_INIT_19));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_68:
            fp_dbg("Reading 68 bytes");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 68);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_0860:
            fp_dbg("Sending 0860 command");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_0860));
            memcpy(transfer->buffer, CMD_INIT_0860, sizeof(CMD_INIT_0860));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_2:
            fp_dbg("Reading 2 bytes");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 2);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_0780:
            fp_dbg("Sending 0780 command");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_0780));
            memcpy(transfer->buffer, CMD_INIT_0780, sizeof(CMD_INIT_0780));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_6:
            fp_dbg("Reading 6 bytes");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 6);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_75:
            fp_dbg("Sending 0x75");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_75));
            memcpy(transfer->buffer, CMD_INIT_75, sizeof(CMD_INIT_75));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_10:
            fp_dbg("Reading 10 bytes");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 10);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_01_AGAIN:
            fp_dbg("Sending 0x01 (again)");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_01));
            memcpy(transfer->buffer, CMD_INIT_01, sizeof(CMD_INIT_01));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_38_AGAIN:
            fp_dbg("Reading 38 bytes (again)");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 38);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_19_AGAIN:
            fp_dbg("Sending 0x19 (again)");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_19));
            memcpy(transfer->buffer, CMD_INIT_19, sizeof(CMD_INIT_19));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_68_AGAIN:
            fp_dbg("Reading 68 bytes (again)");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 68);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_75_AGAIN:
            fp_dbg("Sending 0x75 (again)");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_INIT_75));
            memcpy(transfer->buffer, CMD_INIT_75, sizeof(CMD_INIT_75));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_10_AGAIN:
            fp_dbg("Reading 10 bytes (again)");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 10);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_LONG1:
            fp_dbg("Sending LONG1 (%d bytes)", CMD_LONG1_SIZE);
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, CMD_LONG1_SIZE);
            memcpy(transfer->buffer, CMD_LONG1, CMD_LONG1_SIZE);
            fpi_usb_transfer_submit(transfer, 10000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_2_LONG1:
            fp_dbg("Reading response for LONG1");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 2);
            fpi_usb_transfer_submit(transfer, 5000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_LONG2:
            fp_dbg("Sending LONG2 (%d bytes)", CMD_LONG2_SIZE);
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, CMD_LONG2_SIZE);
            memcpy(transfer->buffer, CMD_LONG2, CMD_LONG2_SIZE);
            fpi_usb_transfer_submit(transfer, 10000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_2154:
            fp_dbg("Reading 2154 bytes for LONG2");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 2154);
            fpi_usb_transfer_submit(transfer, 10000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_SEND_LONG3:
            fp_dbg("Sending LONG3 (%d bytes)", CMD_LONG3_SIZE);
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, CMD_LONG3_SIZE);
            memcpy(transfer->buffer, CMD_LONG3, CMD_LONG3_SIZE);
            fpi_usb_transfer_submit(transfer, 10000, NULL, write_cmd_cb, ssm);
            break;

        case INIT_STATE_READ_2154_AGAIN:
            fp_dbg("Reading 2154 bytes for LONG3");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 2154);
            fpi_usb_transfer_submit(transfer, 10000, NULL, read_response_cb, ssm);
            break;

        case INIT_STATE_COMPLETE:
            fp_dbg("Init sequence complete");
            self->initialized = TRUE;
            fpi_ssm_mark_completed(ssm);
            break;

        default:
            fp_err("Unknown init state %d", current_state);
            fpi_ssm_mark_failed(ssm, fpi_device_error_new(FP_DEVICE_ERROR_PROTO));
            break;
    }
}

static void init_ssm_complete(FpiSsm *ssm, FpDevice *dev, GError *error)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);

    fp_dbg("Init SSM complete, error: %p", error);

    if (error) {
        fp_err("Init failed: %s", error->message);
        self->initialized = FALSE;
        fpi_device_open_complete(dev, error);
    } else {
        fp_dbg("Device initialized successfully");
        self->initialized = TRUE;
        start_interrupt_polling(self);
        fpi_device_open_complete(dev, NULL);
    }

    g_clear_object(&self->init_ssm);
}

/* ========== Capture SSM ========== */

static void capture_ssm_run(FpiSsm *ssm, FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);
    int current_state = fpi_ssm_get_cur_state(ssm);
    FpiUsbTransfer *transfer;

    fp_dbg("Capture state %d", current_state);

    switch (current_state) {
        case CAPTURE_START:
            fp_dbg("Starting capture sequence");
            self->finger_detected = FALSE;
            self->image_buffer = g_malloc0(IMG_TOTAL_SIZE);
            self->image_buffer_len = IMG_TOTAL_SIZE;
            self->image_bytes_received = 0;
            self->image_status = 0xFFFF;
            fpi_ssm_next_state(ssm);
            break;

        case CAPTURE_SEND_CAPTURE_CMD:
            fp_dbg("Sending capture command (0x5100200000)");
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_CAPTURE));
            memcpy(transfer->buffer, CMD_CAPTURE, sizeof(CMD_CAPTURE));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case CAPTURE_READ_STATUS:
            fp_dbg("Reading status after capture command");
            transfer = fpi_usb_transfer_new(dev);
            transfer->short_is_error = FALSE;  // Разрешаем короткие пакеты
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, 64);
            fpi_usb_transfer_submit(transfer, 5000,
                                    fpi_device_get_cancellable(dev),
                                    read_image_chunk_cb, self);
            break;

        case CAPTURE_WAIT_FINGER_ON:
            fp_dbg("Waiting for finger on...");
            break;

        case CAPTURE_WAIT_FINGER_OK:
            fp_dbg("Waiting for finger OK...");
            break;

        case CAPTURE_SEND_CAPTURE_AGAIN:
            fp_dbg("Sending second capture command");
            g_usleep(500000);
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_OUT, sizeof(CMD_CAPTURE));
            memcpy(transfer->buffer, CMD_CAPTURE, sizeof(CMD_CAPTURE));
            fpi_usb_transfer_submit(transfer, 5000, NULL, write_cmd_cb, ssm);
            break;

        case CAPTURE_READ_IMAGE_HEADER:
            fp_dbg("Reading image from EP 0x%02x", EP_DATA_IN);
            self->image_bytes_received = 0;
            transfer = fpi_usb_transfer_new(dev);
            fpi_usb_transfer_fill_bulk(transfer, EP_DATA_IN, IMG_PACKET_SIZE);
            fpi_usb_transfer_submit(transfer, 5000,
                                    fpi_device_get_cancellable(dev),
                                    read_image_chunk_cb, self);
            break;

        case CAPTURE_READ_IMAGE_DATA:
            fp_dbg("Continuing image read");
            continue_image_read(self);
            break;

        case CAPTURE_PROCESS_IMAGE:
            fp_dbg("Processing image, received %zd bytes", self->image_bytes_received);
            if (self->image_bytes_received >= 18 + IMG_SIZE) {
                fp_dbg("✓ Image complete");
                fpi_ssm_next_state(ssm);
            } else {
                fp_dbg("✗ Incomplete image: %zd bytes", self->image_bytes_received);
                fpi_ssm_mark_failed(ssm,
                                    fpi_device_error_new_msg(FP_DEVICE_ERROR_DATA_INVALID,
                                                             "Incomplete image: %zd bytes",
                                                             self->image_bytes_received));
            }
            break;

        default:
            fp_err("Unknown capture state %d", current_state);
            fpi_ssm_mark_failed(ssm, fpi_device_error_new(FP_DEVICE_ERROR_PROTO));
            break;
    }
}

static void capture_ssm_complete(FpiSsm *ssm, FpDevice *dev, GError *error)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);

    /* МАКСИМАЛЬНО РАННЯЯ ОТЛАДКА */
    g_printerr("!!! capture_ssm_complete ВЫЗВАНА !!!\n");
    g_printerr("!!! ssm: %p, dev: %p, error: %p\n", ssm, dev, error);

    /* ОТЛАДКА: сразу показываем состояние */
    g_printerr(">>> capture_ssm_complete START\n");
    g_printerr(">>> error: %p\n", error);
    g_printerr(">>> self: %p\n", self);
    g_printerr(">>> self->image_buffer: %p\n", self ? self->image_buffer : NULL);
    g_printerr(">>> self->image_bytes_received: %zd\n", self ? self->image_bytes_received : 0);
    g_printerr(">>> self->mode: %d\n", self ? self->mode : -1);

    /* Проверяем условие */
    if (self->image_bytes_received >= 18 + IMG_SIZE) {
        g_printerr(">>> УСЛОВИЕ ВЫПОЛНЕНО: image_bytes_received >= 18 + IMG_SIZE\n");
    } else {
        g_printerr(">>> УСЛОВИЕ НЕ ВЫПОЛНЕНО: %zd < %d\n",
                   self->image_bytes_received, 18 + IMG_SIZE);
    }

    if (self->image_buffer) {
        g_printerr(">>> image_buffer is NOT NULL\n");
    } else {
        g_printerr(">>> image_buffer is NULL\n");
    }

    if (!error) {
        g_printerr(">>> error is NULL\n");
    } else {
        g_printerr(">>> error exists: %s\n", error->message);
    }

    FpImage *image = NULL;
    FpPrint *print = NULL;
    DeviceMode current_mode;
    gboolean has_error = FALSE;
    GError *copy_error = NULL;

    current_mode = self->mode;

    if (!error && self->image_buffer && self->image_bytes_received >= 18 + IMG_SIZE) {
        fp_dbg("Creating image...");
        image = fp_image_new(IMG_WIDTH, IMG_HEIGHT);
        fp_dbg("image created: %p", image);

        if (image) {
            gsize len;
            guint8 *data = (guint8 *)fp_image_get_data(image, &len);
            fp_dbg("image data: %p, len: %zu", data, len);

            g_printerr("!!! DEBUG: data=%p, len=%zu, IMG_SIZE=%d !!!\n", data, len, IMG_SIZE);

            if (data && len == IMG_SIZE) {
                /* Копируем сырые данные */
                memcpy(data, self->image_buffer + 18, IMG_SIZE);
                fp_dbg("Image data copied, first byte: 0x%02x", data[0]);
                fp_dbg("last byte: 0x%02x", data[IMG_SIZE-1]);

                /* Сохраняем сырое изображение */
                FILE *f_raw = fopen("/run/fingerprint_raw.raw", "wb");
                if (f_raw) {
                    fwrite(data, 1, IMG_SIZE, f_raw);
                    fclose(f_raw);
                    g_printerr("!!! SAVED RAW: %d bytes !!!\n", IMG_SIZE);
                }

                /* ===== МИНИМАЛЬНАЯ ВЕРСИЯ - БЕЗ ОБРАБОТКИ ===== */
                fp_dbg("Using raw image data without processing");

                /* ДЕТАЛЬНАЯ ОТЛАДКА ДАННЫХ */
                fp_dbg("=== ДЕТАЛЬНАЯ ОТЛАДКА ДАННЫХ ===");
                gsize debug_len;
                guint8 *debug_data = (guint8 *)fp_image_get_data(image, &debug_len);
                g_printerr("image data pointer: %p, len: %zu\n", debug_data, debug_len);
                g_printerr("first 4 bytes from image: %02x %02x %02x %02x\n",
                           debug_data[0], debug_data[1], debug_data[2], debug_data[3]);

                /* ===== ИСПРАВЛЕННЫЙ КОД: Создание шаблона в формате RAW ===== */
                if (current_mode == MODE_ENROLL) {
                    fp_dbg("MODE_ENROLL: preparing template...");

                    /* Получаем данные изображения */
                    gsize img_len;
                    guint8 *img_data = (guint8 *)fp_image_get_data(image, &img_len);

                    if (img_data && img_len > 0) {
                        g_printerr("✓ Image data size: %zu bytes\n", img_len);

                        /* Создаём шаблон */
                        print = fp_print_new(dev);
                        if (print) {
                            /* Устанавливаем тип RAW (как в официальном драйвере) */
                            fpi_print_set_type(print, FPI_PRINT_RAW);

                            /* Создаём GVariant с данными изображения */
                            GVariant *img_variant = g_variant_new_fixed_array(
                                G_VARIANT_TYPE_BYTE,
                                img_data,
                                img_len,
                                1
                            );

                            /* Определяем номер пальца */
                            FpFinger finger_enum;
                            if (self->finger_name) {
                                finger_enum = finger_name_to_enum(self->finger_name);
                            } else {
                                finger_enum = FP_FINGER_RIGHT_INDEX;
                            }

                            /* Создаём структуру данных как в официальном драйвере */
                            /* ИСПОЛЬЗУЕМ ДРУГОЕ ИМЯ - НЕ "data" */
                            GVariant *print_data = g_variant_new(
                                "(y@ay)",
                                                                 finger_enum,      /* номер пальца */
                                                                 img_variant       /* данные изображения */
                            );

                            /* Записываем данные в шаблон */
                            g_object_set(print, "fpi-data", print_data, NULL);

                            /* Устанавливаем finger name */
                            fp_print_set_finger(print, finger_enum);

                            /* Сохраняем отладочную информацию */
                            FILE *f_debug = fopen("/run/fingerprint_template_info.txt", "w");
                            if (f_debug) {
                                fprintf(f_debug, "Template created at: %ld\n", time(NULL));
                                fprintf(f_debug, "Image size: %zu bytes\n", img_len);
                                fprintf(f_debug, "Finger: %d\n", finger_enum);
                                fclose(f_debug);
                            }

                            g_printerr("\n===========================================\n");
                            g_printerr("✅ Шаблон создан в формате RAW\n");
                            g_printerr("📁 Будет сохранён fprintd в /var/lib/fprint/synaptics_0078/\n");
                            g_printerr("===========================================\n\n");
                        }
                    } else {
                        g_printerr("✗ Failed to get image data\n");
                        has_error = TRUE;
                    }
                }
                /* ===== КОНЕЦ ИСПРАВЛЕННОГО КОДА ===== */

            } else {
                fp_dbg("ERROR: Image data invalid - data=%p, len=%zu", data, len);
                g_clear_object(&image);
                has_error = TRUE;
            }
        } else {
            fp_dbg("ERROR: Failed to create image!");
            has_error = TRUE;
        }
    } else {
        fp_dbg("Conditions not met for image creation:");
        if (error) fp_dbg("  - error exists");
        if (!self->image_buffer) fp_dbg("  - image_buffer is NULL");
        if (self->image_buffer) fp_dbg("  - image_bytes_received: %zd < %d",
            self->image_bytes_received, 18 + IMG_SIZE);
        has_error = TRUE;
    }

    /* Если была ошибка, но нет copy_error, создаем общую ошибку */
    if (has_error && !copy_error) {
        copy_error = fpi_device_error_new_msg(FP_DEVICE_ERROR_GENERAL,
                                              "Driver encountered an error during enrollment");
    }

    fp_dbg("Completing operation, mode: %d, print: %p, image: %p, error: %p",
           current_mode, print, image, copy_error);

    switch (current_mode) {
        case MODE_ENROLL:
            fp_dbg("Calling fpi_device_enroll_complete with print=%p, error=%p", print, copy_error);
            fpi_device_enroll_complete(dev, print, copy_error);

            /* ТЕПЕРЬ можно очистить ресурсы */
            fp_dbg("Cleaning up resources after enroll...");
            self->mode = MODE_NONE;
            g_clear_object(&self->capture_ssm);
            g_clear_pointer(&self->image_buffer, g_free);
            self->image_buffer_len = 0;
            self->image_bytes_received = 0;

            /* ===== БЛОК СОВМЕСТИМОСТИ ===== */

            /* ===== КОНЕЦ БЛОКА ===== */
            break;

        case MODE_VERIFY:
            fp_dbg("Calling fpi_device_verify_complete");
            fpi_device_verify_complete(dev, copy_error);
            g_clear_object(&image);
            break;

        case MODE_CAPTURE:
            fp_dbg("Calling fpi_device_capture_complete with image=%p", image);
            fpi_device_capture_complete(dev, image, copy_error);
            break;

        case MODE_NONE:
            fp_dbg("Completing with MODE_NONE");
            if (image) {
                fpi_device_capture_complete(dev, image, copy_error);
            } else {
                fpi_device_capture_complete(dev, NULL, copy_error);
            }
            break;

        default:
            fp_err("Unknown mode %d", current_mode);
            if (image) {
                fpi_device_capture_complete(dev, image, copy_error);
            } else {
                fpi_device_capture_complete(dev, NULL, copy_error);
            }
            break;
    }

    fp_dbg("=== CAPTURE SSM COMPLETE END ===");
    g_clear_error(&error);
}

static void start_capture_sequence(FpiDeviceSynaptics0078 *self)
{
    FpDevice *dev = FP_DEVICE(self);

    fp_dbg("Starting capture sequence");

    /* Проверяем, нет ли уже активного SSM */
    if (self->capture_ssm) {
        fp_warn("Capture SSM already exists");
        g_clear_object(&self->capture_ssm);
    }

    /* Сбрасываем состояние */
    self->finger_detected = FALSE;
    self->image_buffer = NULL;
    self->image_buffer_len = 0;
    self->image_bytes_received = 0;
    self->image_status = 0xFFFF;

    self->capture_ssm = fpi_ssm_new(dev, capture_ssm_run, CAPTURE_NUM_STATES);
    if (!self->capture_ssm) {
        fp_err("Failed to create capture SSM");
        GError *error = fpi_device_error_new(FP_DEVICE_ERROR_GENERAL);
        DeviceMode current_mode = self->mode;

        self->mode = MODE_NONE;

        switch (current_mode) {
            case MODE_ENROLL:
                fpi_device_enroll_complete(dev, NULL, error);
                break;
            case MODE_VERIFY:
                fpi_device_verify_complete(dev, error);
                break;
            case MODE_CAPTURE:
                fpi_device_capture_complete(dev, NULL, error);
                break;
            case MODE_NONE:
                fp_warn("start_capture_sequence called with MODE_NONE");
                /* Ничего не делаем, просто освобождаем ошибку */
                g_error_free(error);
                break;
            default:
                fp_err("Unknown mode %d in start_capture_sequence", current_mode);
                g_error_free(error);
                break;
        }
        return;
    }

    fpi_ssm_start(self->capture_ssm, capture_ssm_complete);
}

/* ========== Device Operations ========== */

static void fpi_device_synaptics_0078_open(FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);
    GUsbDevice *usb_dev = fpi_device_get_usb_device(dev);
    GError *error = NULL;

    fp_dbg("Opening device");

    if (!usb_dev) {
        fp_err("USB device not available");
        fpi_device_open_complete(dev, fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
        return;
    }

    fp_dbg("Resetting device");
    g_usb_device_reset(usb_dev, NULL);
    g_usleep(500000);

    fp_dbg("Setting configuration");
    g_usb_device_set_configuration(usb_dev, 1, NULL);
    g_usleep(500000);

    if (!g_usb_device_claim_interface(usb_dev, 0, 0, &error)) {
        fp_err("Failed to claim interface: %s", error->message);
        fpi_device_open_complete(dev, fpi_device_error_new(FP_DEVICE_ERROR_PROTO));
        g_error_free(error);
        return;
    }
    fp_dbg("Interface 0 claimed");

    self->init_ssm = fpi_ssm_new(dev, init_ssm_run, INIT_NUM_STATES);
    fpi_ssm_start(self->init_ssm, init_ssm_complete);
}

static void fpi_device_synaptics_0078_close(FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);

    fp_dbg("Closing device - START");

    /* Защита от повторного закрытия */
    if (!self->initialized) {
        fp_dbg("Device already closed");
        fpi_device_close_complete(dev, NULL);
        return;
    }

    fp_dbg("Stopping interrupt polling...");
    if (self->interrupt_transfer) {
        fpi_usb_transfer_unref(self->interrupt_transfer);
        self->interrupt_transfer = NULL;
    }

    fp_dbg("Canceling init_ssm...");
    g_clear_object(&self->init_ssm);  // Без проверки, g_clear_object сам проверяет

    /* НЕ очищаем capture_ssm здесь - он уже очищен в capture_ssm_complete */
    fp_dbg("Canceling init_ssm...");
    g_clear_object(&self->init_ssm);
    self->init_ssm = NULL;  // для надёжности

    fp_dbg("capture_ssm (already cleared in capture_ssm_complete)");
    self->capture_ssm = NULL;  // обязательно!

    fp_dbg("Freeing image buffer...");
    g_clear_pointer(&self->image_buffer, g_free);
    self->image_buffer_len = 0;
    self->image_bytes_received = 0;
    self->mode = MODE_NONE;
    self->initialized = FALSE;

    fp_dbg("Freeing username...");
    g_clear_pointer(&self->username, g_free);

    fp_dbg("Freeing finger_name...");
    g_clear_pointer(&self->finger_name, g_free);

    fp_dbg("Completing close...");
    fpi_device_close_complete(dev, NULL);

    /* Даём время USB-передачам завершиться */
    g_usleep(100000);  // 100ms задержки

    fp_dbg("Closing device - END");
}

static void fpi_device_synaptics_0078_capture(FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);

    fp_dbg("Starting capture");

    if (!self->initialized) {
        fp_err("Device not initialized");
        fpi_device_capture_complete(dev, NULL,
                                    fpi_device_error_new(FP_DEVICE_ERROR_NOT_SUPPORTED));
        return;
    }

    /* Проверяем, не выполняется ли уже операция */
    if (self->mode != MODE_NONE || self->capture_ssm) {
        fp_warn("Device already busy with mode %d", self->mode);
        fpi_device_capture_complete(dev, NULL,
                                    fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
        return;
    }

    self->mode = MODE_CAPTURE;
    start_capture_sequence(self);
}

/* ========== Driver Registration ========== */

static const FpIdEntry id_table[] = {
    { .vid = SYNAPTICS_0078_VID, .pid = SYNAPTICS_0078_PID, .driver_data = 0 },
    { .vid = 0, .pid = 0, .driver_data = 0 }
};

static void fpi_device_synaptics_0078_init(FpiDeviceSynaptics0078 *self)
{
    fp_dbg("Initializing device structure");
    self->initialized = FALSE;
    self->init_ssm = NULL;
    self->capture_ssm = NULL;
    self->interrupt_transfer = NULL;
    self->image_buffer = NULL;
    self->image_buffer_len = 0;
    self->image_bytes_received = 0;
    self->image_status = 0;
    self->finger_detected = FALSE;
    self->mode = MODE_NONE;
    self->username = NULL;
    self->finger_name = NULL;
}

static void fpi_device_synaptics_0078_finalize(GObject *object)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(object);

    fp_dbg("Finalizing device - START");

    if (!self) {
        fp_dbg("self is NULL, skipping");
        return;
    }

    /* Убеждаемся, что все ссылки очищены */
    if (self->interrupt_transfer) {
        fp_dbg("Warning: interrupt_transfer still alive in finalize");
        fpi_usb_transfer_unref(self->interrupt_transfer);
        self->interrupt_transfer = NULL;
    }

    if (self->init_ssm) {
        fp_dbg("Warning: init_ssm still alive in finalize");
        g_clear_object(&self->init_ssm);
    }

    if (self->capture_ssm) {
        fp_dbg("Warning: capture_ssm still alive in finalize");
        g_clear_object(&self->capture_ssm);
    }

    if (self->image_buffer) {
        fp_dbg("Warning: image_buffer still alive in finalize");
        g_clear_pointer(&self->image_buffer, g_free);
    }

    if (self->username) {
        fp_dbg("Warning: username still alive in finalize");
        g_clear_pointer(&self->username, g_free);
    }

    if (self->finger_name) {
        fp_dbg("Warning: finger_name still alive in finalize");
        g_clear_pointer(&self->finger_name, g_free);
    }

    self->image_buffer_len = 0;
    self->image_bytes_received = 0;

    fp_dbg("Finalizing device - END");

    G_OBJECT_CLASS(fpi_device_synaptics_0078_parent_class)->finalize(object);
}

/* ========== fprintd compatibility ========== */

static void fpi_device_synaptics_0078_enroll(FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);
    fp_dbg("Enroll called");

    if (!self->initialized) {
        fp_err("Device not initialized");
        fpi_device_enroll_complete(dev, NULL,
                                   fpi_device_error_new(FP_DEVICE_ERROR_NOT_SUPPORTED));
        return;
    }

    if (self->mode != MODE_NONE) {
        fp_err("Device busy with mode %d", self->mode);
        fpi_device_enroll_complete(dev, NULL,
                                   fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
        return;
    }

    /* Получаем имя пальца из параметров */
    /* В libfprint 1.94.10 это может быть в enroll_data */
    self->finger_name = g_strdup("right-index-finger");

    /* Сохраняем имя пользователя - НО ТОЛЬКО ЕСЛИ ЭТО НЕ ROOT */
    const char *user = g_get_user_name();
    if (user && strcmp(user, "root") != 0) {
        self->username = g_strdup(user);
        fp_dbg("Enroll for user: %s", user);
    } else {
        /* Пробуем получить через logname */
        FILE *p = popen("logname 2>/dev/null", "r");
        if (p) {
            char buf[256];
            if (fgets(buf, sizeof(buf), p)) {
                buf[strcspn(buf, "\n")] = 0;
                if (strlen(buf) > 0 && strcmp(buf, "root") != 0) {
                    self->username = g_strdup(buf);
                    fp_dbg("Enroll for user from logname: %s", buf);
                }
            }
            pclose(p);
        }
    }

    /* НИКАКОГО ЗАПАСНОГО ВАРИАНТА! Если не нашли - оставляем NULL */
    if (!self->username) {
        fp_dbg("Could not determine username, will try in capture_ssm_complete");
    }

    self->mode = MODE_ENROLL;
    start_capture_sequence(self);
}

static void fpi_device_synaptics_0078_verify(FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);
    fp_dbg("Verify called");

    if (!self->initialized) {
        fp_err("Device not initialized");
        fpi_device_verify_complete(dev,
                                   fpi_device_error_new(FP_DEVICE_ERROR_NOT_SUPPORTED));
        return;
    }

    if (self->mode != MODE_NONE) {
        fp_err("Device busy with mode %d", self->mode);
        fpi_device_verify_complete(dev,
                                   fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
        return;
    }

    self->mode = MODE_VERIFY;
    start_capture_sequence(self);
}

static void fpi_device_synaptics_0078_cancel(FpDevice *dev)
{
    FpiDeviceSynaptics0078 *self = FPI_DEVICE_SYNAPTICS_0078(dev);

    fp_dbg("Cancel called");

    GError *cancel_error = g_error_new_literal(
        g_quark_from_static_string("fp-device-error"),
                                               1, /* FP_DEVICE_ERROR_GENERAL */
                                               "Operation cancelled"
    );

    /* Отменяем все активные SSM */
    if (self->init_ssm) {
        fpi_ssm_mark_failed(self->init_ssm, g_error_copy(cancel_error));
        g_clear_object(&self->init_ssm);
    }

    if (self->capture_ssm) {
        fpi_ssm_mark_failed(self->capture_ssm, g_error_copy(cancel_error));
        /* Не очищаем capture_ssm здесь - это сделает колбэк */
    }

    g_error_free(cancel_error);

    /* Отменяем USB transfers */
    GCancellable *cancel = fpi_device_get_cancellable(dev);
    if (cancel && !g_cancellable_is_cancelled(cancel)) {
        g_cancellable_cancel(cancel);
    }
}

static void fpi_device_synaptics_0078_class_init(FpiDeviceSynaptics0078Class *klass)
{
    FpDeviceClass *dev_class = FP_DEVICE_CLASS(klass);
    FpImageDeviceClass *img_dev_class = FP_IMAGE_DEVICE_CLASS(klass);
    GObjectClass *object_class = G_OBJECT_CLASS(klass);

    dev_class->id = "synaptics_0078";
    dev_class->full_name = "Synaptics WBDI 06cb:0078";
    dev_class->type = FP_DEVICE_TYPE_USB;
    dev_class->scan_type = FP_SCAN_TYPE_PRESS;

    /* Правильные features для libfprint */
    dev_class->features = FP_DEVICE_FEATURE_CAPTURE;

    dev_class->enroll = fpi_device_synaptics_0078_enroll;
    dev_class->verify = fpi_device_synaptics_0078_verify;
    dev_class->cancel = fpi_device_synaptics_0078_cancel;

    /* Устанавливаем размеры изображения */
    img_dev_class->img_width = IMG_WIDTH;
    img_dev_class->img_height = IMG_HEIGHT;

    dev_class->id_table = id_table;
    dev_class->open = fpi_device_synaptics_0078_open;
    dev_class->close = fpi_device_synaptics_0078_close;
    dev_class->capture = fpi_device_synaptics_0078_capture;

    object_class->finalize = fpi_device_synaptics_0078_finalize;
}
