# synaptics-06cb-0078-driver
Driver for Synaptics 06cb:0078 fingerprint sensor

## Статус

**Драйвер работает**. Захватывает изображение, находит минуции, создаёт шаблоны.

**fprintd не видит отпечатки**. Это связано с архитектурными ограничениями fprintd и не является ошибкой драйвера.

## Почему fprintd не работает

fprintd версии 1.94.5 требует libfprint версии 1.94.9 или новее. Драйвер собран с libfprint 1.94.10, что удовлетворяет этому требованию, но fprintd не может обнаружить неофициальные драйверы из-за архитектурных ограничений.
**Пути хранения**: fprintd ищет отпечатки не там, где сохраняет

## Установка

```bash
git clone https://github.com/ToLiKNoSoV/synaptics-06cb-0078-driver.git
cd synaptics-06cb-0078
meson builddir --prefix=/usr --libdir=/usr/lib
ninja -C builddir
sudo ninja -C builddir install
sudo ldconfig
