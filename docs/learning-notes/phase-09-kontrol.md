# Phase 9 — PID, Pure Pursuit ve kontrolcüyü kilitleyen üç hata

## 1. Pure pursuit: geometri nereye nişan alacağını, PID ne kadar sert
döneceğini söyler

```
  1. robotu yola PROJEKTE et  -> en yakın nokta, yay uzunluğu s
  2. carrot = s + lookahead yay uzunluğundaki nokta
  3. carrot'un kerterizi = heading hatası
  4. iki PID: biri hız, biri dönüş
```

Saf pure pursuit'in **hafızası yoktur**. Sistematik bir sapma — kalibrasyonu
bozuk tekerlek, eğim, sürten teker — robotu yolundan kalıcı olarak kaydırır.
İntegral terimi bunun içindir.

**Lookahead hızla büyür.** Sabit lookahead ya hızda çok kısadır (robot sürekli
aştığı bir noktayı kovalar → **zikzak**) ya da duruşta çok uzundur (**her
köşeyi keser**).

## 2. Anti-windup opsiyonel değil

Çıkış doygunluğa girdiğinde — ve her zaman girer, aktüatörlerin limiti var —
integratör robotun üzerine hareket edemeyeceği bir hatayı biriktirmeye devam
eder. Hata yön değiştirdiğinde integratör büyük bir değer tutuyordur ve çıkış
doygunluktan **çıkmadan önce bu geri sarılmalıdır**. Sonuç: ayar problemi gibi
görünen ama olmayan uzun bir aşım.

İki savunma:

1. İntegral **terimi** (ham toplam değil) sınırlanır — çıkış biriminde, böylece
   limit `ki` ne olursa olsun aynı şeyi ifade eder.
2. Çıkış, hatanın iteceği yönde doygunken integral **hiç birikmez**:

```cpp
const bool pushing_further =
  (would_saturate_high && error > 0.0) || (would_saturate_low && error < 0.0);
if (!pushing_further) { integral_ = candidate_integral; ... }
```

## 3. Hata #1 — Bir açının türevi, iki açının farkı DEĞİLDİR

Açısal kanal heading hatasını takip eder ve heading hatası **sarar** (wrap).

−3.12 rad'dan +3.10 rad'a kayan bir hata **0.06 rad** değişmiştir, 6.22 rad
değil. Ham farkı türevlemek 20 Hz'de **+124 rad/s²** raporlar; gerçek +1.2.

Tek bir böyle diken çıkışı doyurur. Ve bir yöne dönen robot **hep aynı yönde**
sardığı için, **diken hep aynı işaretlidir** — kendisini üreten dönüşü
besler. Pozitif geri besleme.

Ölçülen (düzeltmeden önce): robot limitinde sürekli döndü, heading hatası tüm
değerlerden geçti, **hiçbir hata sinyali onu durduramadı.**

Düzeltme: `Pid::Gains::error_is_angle`. Türev sarılmış farktan alınır:

```cpp
const double delta = gains_.error_is_angle
  ? arb_core::normalizeAngle(error - previous_error_)
  : (error - previous_error_);
```

## 4. Hata #2 — Rate limiter ÖLÇÜME bağlanmamalı

Bu en pahalıya mal olan hataydı, çünkü belirtisi bir ayar probleminden
ayırt edilemez.

Hız komutu, aktüatörden yapamayacağı bir adım istenmesin diye sınırlanır. Soru:
**neye göre** sınırlanır?

**Ölçülen** hıza bağlamak, plantin kendi aşımını limitleyiciye geri besler.
Gazebo'nun teker kontrolcüsü 2.0 rad/s komutunda ~%11 üstünde oturur:

```
  ölçülen ω    = 2.22 rad/s
  izin verilen = [2.22 − 0.15, 2.22 + 0.15] = [2.07, 2.37]
  mutlak limit = [−2.00, 2.00]
```

İzin verilen bant **tamamen limitin üstünde**. Her istek bu banda kırpılınca
2.07 döner — **−2.0 isteği dahil** — ve son kırpma tam 2.0 verir. 2.0
komutlamak ölçümü 2.22'de tutar. Döngü kapanmıştır ve içindeki hiçbir şey onu
açamaz: **117 saniye boyunca kesintisiz 2.0 rad/s.**

Artık çapa **bir önceki KOMUT**. Tam olarak biliniyor, her zaman limitlerin
içinde, ve slew rate'i gerçekten iddia ettiği şey.

> **Not:** Komut çapa olunca iki kırpmanın **sırası ispatla önemsizleşir**
> (önceki komut zaten mutlak limiti sağladığı için kırpmalar yer değiştirebilir).
> Mutasyon testi hiçbir testin farkı göremediğini doğruluyor — ve **haklı**.
> Bu bir test boşluğu değil, *equivalent mutant*.

## 5. Hata #3 — Carrot, yolun ilk köşesinden değil PROJEKSİYONDAN ölçülür

Phase 8 notundaki hatanın aynısı, bağımsız olarak PID takipçisinde de vardı.
Robot 0.35 m'deki carrot'a sürdü, geçti, sonra **arkasındaki** bir noktaya
nişan aldı → heading hatası ≈ π → yerinde dönüş → 150 saniye.

Testler neden yakalamadı? **Her test yolu robotun etrafında yeniden kuruyordu.**
Robotla kayan bir yolda `path[0]` ile projeksiyon aynı noktadır — yani testler
hatanın görünmez olduğu tek koşulu kuruyordu.

Yakalayan test yolu **dünyada sabit** tutar ve her adımda robot çerçevesine
dönüştürür: `Follower.CarrotAdvancesAlongAPathFixedInTheWorld`.

## 6. Deney tasarımı — basamak yanıtı neden gerekli?

"Yerleşme süresi" ve "aşım" sadece bir **basamak girdisi** için tanımlıdır. Bu
yüzden robot yolun **0.6 m yanında** başlatılır. O ofset basamaktır.

Üç kez yeniden tanımlandı, ve her seferinde ölçüm söyletti:

| Pencere | Sorun |
|---|---|
| tüm koşu | "son bantdan çıkış" **son köşeye** düşüyor → 25.8 s'lik koşuda "22.3 s'de yerleşti" |
| 1.5 m | DWA henüz banda girmemiş → "asla yerleşmedi" vs PID "2.0 s" |
| 4.0 m | DWA son örnekte banttan çıkıyor — **köşeyi önceden görmesi**, doğru davranış |
| **3.0 m** | ✅ ilk düz bacak içinde, her iki kontrolcü için ulaşılabilir |

**Ders:** metriğin tanımı, ölçtüğü şey kadar önemlidir. Yanlış tanımlanmış bir
metrik yanlış cevap vermez — **anlamsız** cevap verir, ve anlamsız olduğu
belli olmaz.

## 7. Ölçülen — PID vs DWA, aynı parkur

| Metrik | PID | DWA |
|---|---|---|
| süre | **25.6 s** | 32.0 s |
| ortalama hız | **0.559 m/s** | 0.444 m/s |
| cross-track RMSE (kalıcı) | **0.084 m** | 0.085 m |
| yerleşme süresi | **2.11 s** | 3.55 s |
| aşım | 4.24 % | **0.00 %** |
| pürüzsüzlük (açısal) | 2.514 rad/s² | **0.234 rad/s²** |
| direksiyon yön değişimi | 40 | **2** |
| miras alınan EKF hatası | 0.130 m | 0.039 m |

**İki kontrolcü kalıcı takip doğruluğunda 1 mm içinde**, her yerde ise devasa
farklı.

En önemli satır **pürüzsüzlük**: sadece hata metriklerine bakan biri bu iki
kontrolcüyü eşdeğer sanar. Hiçbir hata metriği birinin tekerleklerini
testere gibi kullandığını göstermez.

Son satır da önemli: **bir kontrolcü, nerede olduğuna dair tahmini kadar
doğrudur.** 0.13 m'lik EKF hatası, altındaki her takip sayısının tabanıdır.

## Mini görev

> 1. `control.yaml`'da `angular.kd`'yi 0.12 → 0.0 yap ve
>    `scripts/evaluate_navigation.py --mode tracking --controller pid`
>    çalıştır. `overshoot_pct` ve `angular_reversals` ne oluyor? **Neden?**
> 2. `in_place_turn_threshold_rad`'ı 1.0 → 3.0 yap (yani yerinde dönüşü
>    kapat). `settling_time_s` ne oluyor ve DWA'nınkine ne kadar yaklaşıyor?
>    Bu sana DWA'nın neden monoton yakınsadığını söylüyor mu?
> 3. `lookahead_min_m`'i 0.35 → 0.15 yap. `cross_track_steady_rmse_m`
>    düşüyor mu yükseliyor mu — ve `angular_reversals`'a ne oluyor?
>    İkisi arasındaki takas nedir?
