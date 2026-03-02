#include <stdio.h>
#include <libfprint-2/fprint.h>

int main(void)
{
    FpContext *ctx;
    GPtrArray *devices;
    FpDevice *dev = NULL;
    GError *error = NULL;
    FpImage *image = NULL;

    ctx = fp_context_new();
    devices = fp_context_get_devices(ctx);

    for (guint i = 0; i < devices->len; i++) {
        FpDevice *d = g_ptr_array_index(devices, i);
        const char *driver = fp_device_get_driver(d);
        if (g_strcmp0(driver, "synaptics_0078") == 0) {
            dev = g_object_ref(d);
            break;
        }
    }

    g_ptr_array_unref(devices);

    if (!dev) {
        printf("No synaptics_0078 device found\n");
        return 1;
    }

    printf("Opening device...\n");
    if (!fp_device_open_sync(dev, NULL, &error)) {
        printf("Failed to open: %s\n", error->message);
        return 1;
    }

    printf("Place finger on sensor...\n");
    image = fp_device_capture_sync(dev, 30, NULL, &error);

    if (image) {
        gsize len;
        const guint8 *data = fp_image_get_data(image, &len);
        printf("Captured %zu bytes\n", len);
        
        FILE *f = fopen("/run/fingerprint_raw.raw", "wb");
        if (f) {
            fwrite(data, 1, len, f);
            fclose(f);
            printf("Saved to /run/fingerprint_raw.raw\n");
        }
        
        g_object_unref(image);
    } else {
        printf("Capture failed: %s\n", error->message);
    }

    fp_device_close_sync(dev, NULL, NULL);
    g_object_unref(dev);
    g_object_unref(ctx);

    return 0;
}
