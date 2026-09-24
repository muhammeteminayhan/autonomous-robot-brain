# Phase 11 — Benchmark: Tek ölçüm ölçüm değildir

## 1. Phase 10'un asıl zayıflığı neydi?

Phase 10 her arıza türünü **bir kez**, **tek bir genlikte** koştu. Bu şu soruyu
cevaplar: "bu arıza türü teorinin dediğini yapıyor mu?"

Cevaplayamadığı iki soru var, ve sırada tam olarak onlar var:

1. **Kestirim ne kadarını kaldırabiliyor?** Bir eğrinin üzerindeki tek nokta,
   eğrinin nerede kırıldığını söylemez.
2. **Bu bir etki mi, gürültü mü?** Phase 10'da Case A'nın kovaryans tutarlılığı
   %76, Case D'nin %92 çıktı — ve aradaki fark, ikisinin **farklı koşular**
   olmasıydı.

Çözüm: tek seferde **tek parametre**, bir aralık boyunca **taranmış**, her nokta
**üç tohum** üzerinden **tekrarlanmış**, ortalama ± yayılım olarak raporlanmış.

## 2. `±` bir standart HATA değil, standart SAPMA

Buradaki `±`, tekrarların birbirinden ne kadar farklı olduğu — **popülasyon
standart sapması**.

Kasıtlı olarak standart hata veya güven aralığı **değil**: üç örnek bunu
taşıyamaz, ve `± 0.002`'yi gerçek ortalamayı sınırlıyormuş gibi yazmak, var
olmayan bir hassasiyet iddia etmek olur.

> **Üç tekrar azdır.** Bir etkinin saçılmadan büyük olup olmadığını görmeye
> yeter; bir dağılımın kuyruğu hakkında konuşmaya yetmez.

## 3. Bir koşu neye benzer?

```
temiz simülatör -> temiz EKF -> temiz harita -> sabit AÇIK ÇEVRİM rota
  -> tek arıza, tek genlik -> ground truth'a karşı puanla
```

Rota, zamanlanmış hız komutlarıyla sürülüyor. Arızalar simülatörün
**aşağısındaki** ön-işleme katmanına enjekte ediliyor, yani tekerlekler aynı
komutları alıyor ve **gerçek yörünge bir taramanın her koşusunda aynı.**
Sonuçtaki her fark kestirim hatasıdır.

## 4. Neden her koşuda HER ŞEY yeniden başlıyor?

Daha ucuz seçenekler düşünüldü ve hepsi yanlış:

- **Sadece sensör node'larını yeniden başlatmak**: EKF önceki koşunun
  durumunu, mapping node'u önceki koşunun **haritasını** tutmaya devam eder.
  N. koşu N-1'in hatasını miras alır — arıza zamanlayıcısının bir koşu
  *içinde* önlemek için uğraştığı kirlenmenin tam olarak aynısı.
- **Tohumları runtime'da değiştirmek**: imkânsız. Gürültü üreteçleri tohumla
  **inşa ediliyor**, ve node'lar bu denemeyi **reddediyor** — parametre
  sunucusunun çalışan modelle çelişmesine izin vermek yerine.

Bu yüzden her tekrar, üzerine yazılmış bir parametre dosyasıyla yeniden
başlıyor.

### Tohumlar ayrı ayrı ötelenıyor, ortak bir değere set edilmiyor

Jiroya ve encoder'lara **aynı** tohumu vermek, gürültü akışlarını
**korele** eder. Jiro gürültüsü ile teker gürültüsü birlikte hareket eden bir
robot, diğer bütün fazların ölçtüğü robottan **fiziksel olarak farklı** bir
robottur.

## 5. Bu fazın, koşmadan önce bulduğu hata

Jiro taraması **0.5x**'ten 20x'e gidiyor — yanıtın iki ucunu birden görmek için
kasıtlı. Alt uç **hiçbir şey ölçmüyordu.**

`knobs_at` her arızayı mevcut knob kümesine karşı `max(mevcut, istenen)` ile
kompoze ediyordu, ve **nominalden** başlıyordu. O kural **iki üst üste binen
arızayı** çözmek için var (Case F'in bütün amacı); bir arıza ile **temel
çizgi** arasına uygulandığında, nominalden **azını** isteyen her şeyi sessizce
kırpıyor:

```
max(1.0, 0.5)  ==  1.0
```

Datasheet gürültüsünün yarısındaki altı koşu, **sağlıklı robotu** ölçecekti ve
her grafiğin solunda düz bir çizgi üretecekti — ki bu tam olarak bir
"azalan getiri" eğrisine benziyor. Hiçbir şey uyarı vermeyecekti.

**Taramanın senaryolarını üretip, koşturmadan ÖNCE okuyarak** bulundu. Bu iki
saatlik simülasyondan daha değerli: yanlış bir sayıyı bir sözlükte görmek, bir
grafikte görmekten kolaydır.

> **Ders:** Üreteceğin deneyin girdisini, deneyi koşturmadan önce oku.

## 6. Beşinci deney kestirimi değil PLANLAYICIYI test ediyor

Dört tarama bir **sensörü** bozuyor. Son deney **dünyayı** bozuyor: robotun
sürdüğü koridoru bir engel geçiyor, ve o engel **haritada yok.**

Her tohum **iki kez** koşuyor — engelli ve engelsiz — ve çiftler
karşılaştırılıyor. Mutlak varış süresi tek başına az şey söyler, çünkü harita
doldukça rota yeniden planlanıyor ve iki koşu tam aynı yolu izlemiyor.
Eşleştirilmiş bir çift arasındaki **fark**, engelin maliyetidir.

Engelle ilgili üç şey kasıtlı, ve her biri önce yanlış yapıldı:

| Karar | Neden alternatifi yanlış? |
|---|---|
| Çalışan dünyaya **spawn** ediliyor, kat planına yazılmıyor | Başlangıçta var olan bir engel yol üstünde haritalanır ve A* etrafından geçer — bu costmap'i test eder, **tepki vermeyi** değil |
| **Collision geometrisi** olan bir model, Gazebo `<actor>` değil | Actor'lar varsayılan olarak **sadece görsel**: collision yok, LiDAR içinden geçip görür. Sensörün algılayamadığı bir engel hiçbir şeyi test etmez |
| Her tick **teleport** ediliyor, hız kontrollü değil | Teleport, yörüngeyi simüle zamanın **tam** bir fonksiyonu yapar ve tohumlar arası **birebir** tekrarlanır. Hız kontrollü model kendi integrasyon farklarını biriktirir, robot her koşuda engelle biraz **başka bir yerde** buluşur — ve tekrarlar arasında aynı kalması gereken tek şey, yeni bir varyans kaynağına dönüşür |

Engelin gerçek pozu **enstrümantasyon** olarak yayınlanıyor ve ground truth ile
fault state ile **aynı izolasyon kuralına** tabi: bir lokal planlayıcıya engelin
nerede olduğunu **söylemek**, onu **görmesini** sağlamaktan farklı ve çok daha
kolay bir problemdir.

## 7. Sonuçlar

Tablolar: [`docs/benchmarks.md`](../benchmarks.md) bölüm 4, ve
`results/phase11/benchmark_report.md`.

## 8. Bu taramaların söyleyemediği şeyler

- **Üç tohum.** Bir etkinin saçılmayı aşıp aşmadığını görmeye yeter; bir
  dağılımı ya da en kötü durumu karakterize etmeye yetmez.
- **Tek seferde tek parametre.** Etkileşimler tam olarak Case F'in konusuydu ve
  tek eksenli bir tarama onlar hakkında hiçbir şey söylemez.
- **Tek ortam.** Her koşu aynı 20x14 m dünyada, aynı spawn'dan. Daha seyrek bir
  ortam LiDAR sonuçlarını ciddi biçimde değiştirir — likelihood field'ın iki
  düz duvarlı bir koridorda kilitleneceği çok daha az şey var.
- **Simülatörün sensör modelleri idealize.** Mutlak sayılar **birbirlerine
  göre** anlamlı, gerçek donanım tahmini olarak değil.

## Mini görev

> 1. `configs/benchmarks.yaml`'da `seeds`'i `[1, 2, 3, 4, 5, 6]` yap ve
>    `encoder_bias` taramasını tekrar koştur. `±` bandı **daralıyor mu
>    genişliyor mu**? Neden? (Popülasyon sapması ile standart hata arasındaki
>    farkı düşün.)
> 2. `duration_s`'i 75 → 150 yap. Hangi sütun değişiyor ve hangisi
>    değişmiyor — ve bu sana dead reckoning hatasının neyle büyüdüğünü
>    söylüyor mu?
> 3. `lidar_dropout` taramasında `burst_mode: true` yap (aynı ortalama
>    dropout oranıyla). Sonuç aynı mı? Eğer değilse: aynı miktarda kayıp ışın,
>    neden farklı bir sonuç veriyor?
