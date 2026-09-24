# Phase 5 — Değerlendirme: Bir kestirimci nasıl DÜRÜSTÇE ölçülür?

> Bu not eksikti — indeks onu listeliyordu ama dosya hiç yazılmamıştı. Phase 5
> Phase 4 ile aynı commit'te geldiği için not atlanmış. İçerik ise atlanacak
> içerik değil: bu fazın fikirleri, Phase 10 ve 11'deki her ölçümün altında
> duruyor.

## 1. Neyi ölçüyoruz, ve neye karşı?

Ground truth Gazebo'dan geliyor: robotun **gerçekten** nerede olduğu.
Kestirimcinin **nerede olduğunu sandığı** ile arasındaki fark, hata.

> **Kural:** Ground truth **sadece** puanlamak için kullanılır, asla kestirmek
> için. `tests/integration/test_ground_truth_isolation.py` bunu statik olarak
> (kaynak taraması) ve runtime'da (canlı grafikte abone taraması) zorluyor.

## 2. En kritik karar: yörüngeyi HİZALAMAYIN

Her kestirimci kendi orijininden başlıyor; ground truth robotun spawn
edildiği yerden. İkisini karşılaştırmak için bir hizalama gerekiyor.

Cazip olan şey: tüm yörüngeyi en iyi uyacak şekilde **fit** etmek (Umeyama /
en küçük kareler ile rijit dönüşüm). **Yapma.**

```
YANLIŞ:  tüm yörüngeyi fit et  -> sürüklenmenin bir kısmı fit'e emilir
DOĞRU:   SADECE ilk örneği hizala, rijit dönüşüm, fit yok
```

Tüm yörüngeyi fit etmek, sürüklenmenin bir kısmını hizalamanın içine emer ve
kestirimciyi **kayırır**. Bir arıza enjeksiyon deneyinde bu, **tam olarak
ölçmeye çalıştığın şeyi** emer.

Kodda: `scripts/evaluate_localization.py` → `align_and_diff`, ve
`scripts/run_scenario.py` → `align`.

## 3. Doğru normalizasyon: mesafeye göre, zamana göre değil

Dead reckoning hatası **kat edilen mesafeyle** büyür, geçen süreyle değil.
Duran bir robot sonsuza kadar durabilir ve hatası büyümez.

Bu yüzden ana metrik:

```
position_final_pct_of_distance = 100 * son_hata / kat_edilen_mesafe
```

"10 metrede 9 cm" ile "100 metrede 9 cm" aynı sayı değil, ve zamanla
normalize etmek ikisini aynı gösterir.

> Bu, Phase 11'de tekrar ısırdı: oradaki koşular 75 s, Phase 10'unkiler 120 s.
> Mutlak sayılar **karşılaştırılabilir değil**, ve rapor bunu söylüyor.

## 4. Ölçülen — Phase 5 baseline

10.03 m'lik rota, `odometry_test` pozundan:

| Metrik | Odometri | EKF | İyileşme |
|---|---|---|---|
| Pozisyon RMSE | 0.0486 m | **0.0155 m** | **−68.0 %** |
| Mesafenin yüzdesi (son hata) | 0.943 % | **0.134 %** | −86 % |
| Yön RMSE | 3.63° | **1.43°** | **−60.6 %** |
| Yön maks. | 6.81° | 3.22° | |
| Raporlanan son 1σ (pozisyon) | 0.244 m | 0.117 m | |
| 2σ içinde kalan örnekler | %100 | %100 | |

## 5. Kovaryans tutarlılığı — "filtre yalan söylüyor mu?"

Bir filtre iki şey üretir: bir **tahmin** ve o tahmine **ne kadar güvendiği**.
İkincisi kontrol edilmezse ilki tek başına yanıltıcıdır.

`fraction_inside_2sigma_pos`: hatanın, filtrenin kendi 2σ'sının içinde kaldığı
örnek oranı. İyi ayarlanmış bir filtrede **~%95** olmalı.

- **%100** → filtre fazla **temkinli**. Sınırı doğru ama o kadar geniş ki
  hiçbir şeyi kısıtlamıyor. Odometri burada: 0.244 m'lik 1σ ile 0.049 m'lik
  gerçek hata. Tutarlı ve işe yaramaz.
- **%95** → dürüst.
- **%50** → filtre fazla **kendine güveniyor**, ve bu **kötü bir filtreden
  daha kötü**: aşağı akıştaki her şey ona inanır.

> Phase 10 tam olarak bu metriği kullanarak asıl kusuru buldu: Case C'de EKF
> yönünü ±0.31° güvenle raporlarken 12° yanlıştı. Phase 5'te %100 çıkması, o
> koşuda hiçbir arıza olmamasıydı — arıza gelince metrik %47'ye düştü.

## 6. Gecikme — tahmin ne kadar bayat?

| | ms |
|---|---|
| medyan | 55.0 |
| ortalama | 56.3 |
| p95 | 60.0 |
| maks | 70.0 |

Yayınlanan tahmin, **üzerine kurulduğu verinin yaşı** kadar geride. EKF bunu
şu anki zamana **ileri tahmin** ederek (`publish lead 350 ms`) telafi ediyor —
yoksa kontrolcü 55 ms önceki bir poza göre direksiyon kırardı, ve 0.5 m/s'de
bu 2.75 cm'lik sistematik bir cross-track hatası demek.

## 7. Neden dikdörtgen rota?

**Dönüşler dead reckoning'i bozan şeydir.** Sadece düz giden bir rota her
kestirimciyi kayırır:

- Düz çizgide encoder hatası mesafeyle lineer birikir — küçük.
- Dönüşte teker yarıçapı ve iz genişliği hataları **açıya** dönüşür, ve bir
  açı hatası sonrasındaki **tüm** mesafeyi yanlış yöne taşır.

Phase 3 bunu ölçtü: iz genişliği kalibrasyonundan önce sürüklenme mesafenin
%9.86'sı, sonra %0.94'ü. O 10 kat, tamamen dönüşlerde.

## Mini görev

> 1. `scripts/evaluate_localization.py`'da `align_and_diff` içindeki
>    hizalamayı, **son** örneğe hizalayacak şekilde değiştir (ilk yerine).
>    `position_rmse_m` ne oluyor? Bu sayı hangi anlamda "daha iyi" ve neden
>    **dürüst değil**?
> 2. `odometry.launch.py`'ı `ekf_use_imu:=false` ile koştur. Yön RMSE'si
>    odometriye ne kadar yaklaşıyor? Jiro'nun katkısı tam olarak nedir?
> 3. `fraction_inside_2sigma_pos` odometri için %100. EKF'in
>    `position_process_sigma`'sını 10 katına çıkar ve tekrar koştur: metrik
>    %100'e çıkıyor mu, ve **pozisyon RMSE'si** ne yapıyor? Aradaki takas ne?
