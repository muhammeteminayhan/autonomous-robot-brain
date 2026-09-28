# Phase 12 — Dokümantasyon: Doğrulanmamış bir sayı, yanlış bir sayıdır

Bu faz kod yazmıyor. Yaptığı şey, on bir fazın ürettiği iddiaları **kontrol
etmek** — ve kontrol edince çıkanlar, bu notun asıl konusu.

## 1. Bulunan tutarsızlıklar

| Nerede | İddia | Gerçek |
|---|---|---|
| `README.md` | "273 unit, 94 integration test" | **332 / 126** — Phase 9'dan beri bayat |
| `docs/project-report.md` | tabloda 332, CV bölümünde 330 | Kendi kendisiyle çelişiyor |
| `README.md` | "60 mutasyon" (öncesinde 41) | Denetlenmemiş sayı |
| `docs/architecture.md` | "Phase 12 composed-container launch ekler" | Eklemedi |
| `docs/learning-notes/README.md` | `phase-05-degerlendirme.md` linki | Dosya **hiç yazılmamıştı** |
| `.gitignore` | `results/**/raw/` | Metrikleri sessizce dışlıyordu |

Hiçbiri testleri kırmıyordu. Hepsi de, okuyan birine yanlış bilgi veriyordu.

## 2. Test sayısını "hesaplamaya" çalışmak

Süpürmeler koşarken CPU'yu meşgul etmemek için, testleri **çalıştırmadan**
saymayı denedim: her gtest binary'sinde `--gtest_list_tests`.

Sonuç **319**. Gerçek sayı **332**.

Fark neden? `colcon` sadece gtest vakalarını değil, paket başına **lint
testlerini** de (uncrustify, xmllint, lint_cmake) koşuyor ve sayıyor. Bunlar
gtest vakası değil, dolayısıyla listelenemiyorlar.

> **Ders:** Bir ölçümün kısayolu, ölçtüğünü sandığın şeyi ölçmeyebilir.
> 319 makul görünüyordu — ve makul görünen yanlış sayı, saçma görünen yanlış
> sayıdan daha tehlikelidir.

Doğru sayı için testleri gerçekten koşmak gerekti. Süpürmeler bitene kadar
bekledim ve bu arada dört yanlış sayının **yanlış olduğunu açıkça not ettim**,
sessizce bırakmak yerine.

## 3. `.gitignore`'da sondaki eğik çizgi

```gitignore
results/**/raw/     # YANLIŞ
results/**/raw/*    # doğru
```

Git, **dışlanmış bir DİZİNİN içine girmez**. Sondaki eğik çizgiyle, altındaki
bütün `!` negatifleri **ulaşılamaz** hale geliyordu — ve koşu başına metrikler,
ayrılmaları gereken hacimli verilerle birlikte sessizce düşüyordu.

O JSON'lar **kayıttır**: her tablo ve grafik onlardan türetiliyor ve
`--aggregate-only` bütün raporu simülatöre dokunmadan yeniden üretiyor. Onlarsız
bir klone, tek bir sayıyı ne doğrulayabilir ne düzeltebilir.

## 4. Ekran görüntüsü: araç yoksa formatı kendin oku

Makinede ImageMagick, netpbm, ffmpeg yok; sadece `xwd` ve PIL var — ve PIL XWD
okumuyor.

Çözüm: XWD formatını elle parse etmek. 100 baytlık big-endian başlık, pencere
adı, colormap, sonra ZPixmap verisi. ~40 satır.

Yol boyunca iki şey:

- `xwd -root` Gazebo GUI açıkken **`BadColor`** ile patlıyor — GUI kendi
  colormap'ini kuruyor. Çözüm: kök pencere yerine **pencereyi id ile** yakala
  (`xwininfo -root -children` ile bul).
- Gazebo'nun kamerası **+X ekseni boyunca** bakıyor, +Y değil. Benim
  quaternion'um +Y varsayıyordu ve kamera gökyüzüne döndü.

## 5. Bu fazın asıl dersi

On bir faz boyunca her sayıyı ölçtüm ve kaydettim. Yine de dokümantasyonda
**dört yanlış test sayısı**, **bir ölü link**, **bir tutulmamış söz** ve
**kaydı sessizce atan bir `.gitignore` kuralı** birikmişti.

> Kod testlerle korunuyordu. **Dokümantasyonu hiçbir şey korumuyordu.**

`tests/integration/test_docs_consistency.py` bunu değiştiriyor: her göreli link
çözülmeli, öğrenme notları indeksi ile dizin **iki yönde de** uyuşmalı, faz
numaraları kesintisiz olmalı. Üçü de, yakalaması gereken hatalara karşı
enjeksiyonla doğrulandı.

Hâlâ koruma altında **olmayan** şey: bir sayının **doğru** olup olmadığı. Test
`332`'nin bir sayı olduğunu görür, doğru sayı olduğunu değil. Bunun için
ölçümü koşturmak ve okumak gerekiyor — ve bu fazın yaptığı buydu.

## Mini görev

> 1. `docs/` altındaki herhangi bir `.md` dosyasına var olmayan bir dosyaya link
>    ekle ve `pytest tests/integration/test_docs_consistency.py` koştur.
>    Hangi test kırılıyor, ve hata mesajı **hangi dosyayı** söylüyor?
> 2. `.gitignore`'daki `results/**/raw/*` kuralını `results/**/raw/` yap ve
>    `git check-ignore -v results/phase11/raw/encoder_bias_lvl0_seed1.json`
>    çalıştır. Çıktı ne diyor, ve `!results/**/raw/*.json` neden artık işe
>    yaramıyor?
> 3. `docs/project-report.md`'deki test sayısını 999 yap. **Hangi test bunu
>    yakalar?** (İpucu: hiçbiri. Neden yakalayamaz, ve yakalayabilmesi için ne
>    gerekirdi?)
