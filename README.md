# pico-serprog-avologame

Builder repo untuk eksperimen firmware `pico-serprog` berbasis RP2040/Pico.

Repo ini disiapkan untuk dua profil firmware:

- `stable-4096`: fokus stabil untuk penggunaan normal
- `benchmark-8192`: fokus uji throughput lebih tinggi

Target repo ini:
- build firmware `.uf2` lewat GitHub Actions
- memudahkan A/B test antar profil
- menyimpan modifikasi terhadap firmware upstream `opensensor/pico-serprog`

## Cara pakai

Setelah workflow berhasil, ambil artifact build dari tab **Actions**.

Untuk flash ke Pico:
1. tekan dan tahan tombol **BOOTSEL**
2. colokkan Pico ke USB
3. lepaskan tombol saat drive **RPI-RP2** muncul
4. salin file `.uf2` hasil build ke drive tersebut

## Catatan

Firmware ini turunan dari proyek `opensensor/pico-serprog` dan tetap mengikuti lisensi kode asalnya.
