# Audit VESC 6.00 Runtime V16

## Root cause getar setelah Hall detect

Tiga hasil Hall detect hardware yang dilaporkan user konsisten, sehingga masalah utama bukan EEPROM/random mapping. V15 menggunakan nilai tabel Hall (sector center) sebagai base interpolation segera setelah edge, kemudian menambahkan hingga 60° electrical lagi. Hal ini dapat memberi phase advance berlebih. Threshold interpolation adalah 30 RPM mechanical; dengan 15 pole-pair nilainya 450 ERPM, sama dengan area gejala yang sebelumnya muncul.

V16 menyimpan midpoint old/new Hall center sebagai edge position, menginterpolasi dari edge tersebut berdasarkan Hall period, dan memberi wrapped phase rate-limit.

## Current saat OFF

Log hardware menunjukkan Id/Iq puluhan ampere saat state OFF dan duty 0. V16 menganggap ADC motor current tidak valid saat bridge undriven dan meng-zero-kan alpha/beta, Id/Iq, Iin dan LPF current pada OFF branch. Proteksi raw over-current ISR tetap independen dan tidak dihilangkan.

## Realtime VESC Tool

V15 hanya memiliki satu pending-packet buffer, sehingga burst request dapat hilang sebelum main loop memprosesnya. V16 memakai FIFO depth 4. GET_VALUES normal/selective dan GET_VALUES_SETUP normal/selective selalu reply segera dan menyimpan tipe/mask request realtime terakhir untuk frame periodik 20 ms selama link aktif. Legacy telemetry tidak dikirim selama VESC binary link aktif.

## Hall detect

Detector V16 menggunakan alignment-current ramp 1 s lalu tiga sweep forward dan tiga reverse, masing-masing 0..359° electrical dengan step 1° dan dwell 5 ms. Enam raw Hall state harus memiliki >30 sample dan sector spacing tetap divalidasi sebelum tabel diterapkan/disimpan.
