# Авионика проекта

Для выполнения проекта был выбран инерциальный измерительный модуль (IMU) - ICM-42688-P. (https://static.chipdip.ru/lib/976/DOC031976605.pdf) К его достоинствам стоит отнести:

* **Низкий уровень шума** – по заявленным характеристикам и реальным тестам, уровень шума гироскопа на данной модели составляет всего 2.8 mdps/√Hz (градусов в секунду на корень частоты), а акселерометра порядка 70 μg/√Hz (g ~ 9.81 м/с^2)
* **Высокая температурная стабильность** – датчик очень слабо подвержен влиянию температуры, измерения практически не страдают от температуры окружающей электроники и внешнего температурного режима
* **Толерантность к механическому резонансу двигателей** – модули датчика работают при диапазоне частот в 25-29 kHz, частота работы моторов гораздо ниже типового значения, что полностью исключает влияние вибраций на итоговые показатели датчиков.
* **Минимальные массогабаритные параметры** – датчик весит доли грамма и потребляет единицы миллиампер.

---

## Расчёт памяти

В даташите модуля указаны объем информации, принимаемый одной осью гироскопа и акселерометра с одной оси за такт – 16 бит. Имея данное значение можно рассчитать примерный объём памяти необходимый для работы данного модуля на всё время идеализированного полёта:

Акселерометр и гироскоп на данном модуле трёхосные ($a_x, a_y, a_z$) и ($\omega_x, \omega_y, \omega_z$): 
* 3 оси * 2 байта = 6 байт для каждого датчика
* временная метка = 2 байта
* информация о температуре кристалла = 2 байта

$B$ – один кадр измерений  
$B = 6\text{ байт} * 2 + 2\text{ байта} + 2\text{ байта} = 16\text{ байт}$

Для базовой частоты съёма кадра измерений: $F = 200\text{ Hz}$  
За одну секунду: $R = 16\text{ байт} * 200\text{ с}^{-1} = 3200\text{ байт/с} = 3.125\text{ КБ/с}$  
За одну минуту: $V = 3200\text{ байт/с} * 60\text{ с} = 192\,000\text{ байт} \approx 187.5\text{ КБ}$

### Сценарий 1
Предположим, что прямое прохождение маршрута без задержки занимает 25 минут ($1500\text{ с}$), тогда:  
$V_{\text{ном}} = 3200\text{ байт/с} * 1500\text{ с} = 4\,800\,000\text{ байт} \approx 4.58\text{ МБ}$

### Сценарий 2
По ТЗ на пути 7 препятствий, требующих маневра:
* Запретная зона $+20\%$ длины пути;
* 3 Вертикальных облёта: $3 * 4\% = +12\%$ пути;
* 2 Горизонтальных облёта: $5\% + 6\% = +11\%$ пути;
* Пролёт под мостом: $+4\%$ пути.

Суммарное удлинение пути составляет:  
$$\Delta L = 20\% + 12\% + 11\% + 4\% = +47\%\text{ к длине прямого пути}$$

Также по ТЗ необходимо снижать скорость перед сложными объектами, при учёте падения средней скорости получим добавку к времени полёта порядка $+60\%$ от первого сценария.  
$T_{\text{доп}} = 1500\text{ с} \times 0.60 = 900\text{ с} = 15\text{ минут}$  
$V_{\text{доп}} = 3\,200\text{ байт/с} \times 900\text{ с} = 2\,880\,000\text{ байт} \approx 2.75\text{ МБ}$

Получим итоговый объём миссии:  
$V_{\text{миссия}} = 4.58\text{ МБ} + 2.75\text{ МБ} = 7.33\text{ МБ}$

### Сценарий 3
Максимальное время нахождения в воздухе Mavic 3 Pro со штатным АКБ составляет – 43 минуты ($T_{\text{макс}} = 43 \times 60 = 2580\text{ с}$):  
$V_{\text{макс}} = 3\,200\text{ байт/с} \times 2580\text{ с} = 8\,256\,000\text{ байт} \approx 7.87\text{ МБ}$

В реальности бортовая память выбирается с двукратным запасом на случай повторного вылета без предварительной очистки логов, задержки посадки или других непредвиденных случаев:  
$V_{\text{проект}} = V_{\text{макс}} \times 2 \approx 15.74\text{ МБ}$

Это минимально необходимая память для хранения телеметрии чипа.

---

Информация по объемам памяти взята из регистровой карты модуля:

| Addr (Hex) | Addr (Dec.) | Register Name | Serial I/F | Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0 |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| 11 | 17 | DEVICE_CONFIG | R/W | - | - | - | SPI_MODE | - | - | - | SOFT_RESET_CONFIG |
| 13 | 19 | DRIVE_CONFIG | R/W | - | - | I2C_SLEW_RATE | I2C_SLEW_RATE | - | SPI_SLEW_RATE | SPI_SLEW_RATE | SPI_SLEW_RATE |
| 14 | 20 | INT_CONFIG | R/W | - | - | INT2_MODE | INT2_DRIVE_CIRCUIT | INT2_POLARITY | INT1_MODE | INT1_DRIVE_CIRCUIT | INT1_POLARITY |
| 16 | 22 | FIFO_CONFIG | R/W | FIFO_MODE | FIFO_MODE | - | - | - | - | - | - |
| 1D | 29 | TEMP_DATA1 | SYNCR | TEMP_DATA[15:8] | TEMP_DATA[15:8] | TEMP_DATA[15:8] | TEMP_DATA[15:8] | TEMP_DATA[15:8] | TEMP_DATA[15:8] | TEMP_DATA[15:8] | TEMP_DATA[15:8] |
| 1E | 30 | TEMP_DATA0 | SYNCR | TEMP_DATA[7:0] | TEMP_DATA[7:0] | TEMP_DATA[7:0] | TEMP_DATA[7:0] | TEMP_DATA[7:0] | TEMP_DATA[7:0] | TEMP_DATA[7:0] | TEMP_DATA[7:0] |
| 1F | 31 | ACCEL_DATA_X1 | SYNCR | ACCEL_DATA_X[15:8] | ACCEL_DATA_X[15:8] | ACCEL_DATA_X[15:8] | ACCEL_DATA_X[15:8] | ACCEL_DATA_X[15:8] | ACCEL_DATA_X[15:8] | ACCEL_DATA_X[15:8] | ACCEL_DATA_X[15:8] |
| 20 | 32 | ACCEL_DATA_X0 | SYNCR | ACCEL_DATA_X[7:0] | ACCEL_DATA_X[7:0] | ACCEL_DATA_X[7:0] | ACCEL_DATA_X[7:0] | ACCEL_DATA_X[7:0] | ACCEL_DATA_X[7:0] | ACCEL_DATA_X[7:0] | ACCEL_DATA_X[7:0] |
| 21 | 33 | ACCEL_DATA_Y1 | SYNCR | ACCEL_DATA_Y[15:8] | ACCEL_DATA_Y[15:8] | ACCEL_DATA_Y[15:8] | ACCEL_DATA_Y[15:8] | ACCEL_DATA_Y[15:8] | ACCEL_DATA_Y[15:8] | ACCEL_DATA_Y[15:8] | ACCEL_DATA_Y[15:8] |
| 22 | 34 | ACCEL_DATA_Y0 | SYNCR | ACCEL_DATA_Y[7:0] | ACCEL_DATA_Y[7:0] | ACCEL_DATA_Y[7:0] | ACCEL_DATA_Y[7:0] | ACCEL_DATA_Y[7:0] | ACCEL_DATA_Y[7:0] | ACCEL_DATA_Y[7:0] | ACCEL_DATA_Y[7:0] |
| 23 | 35 | ACCEL_DATA_Z1 | SYNCR | ACCEL_DATA_Z[15:8] | ACCEL_DATA_Z[15:8] | ACCEL_DATA_Z[15:8] | ACCEL_DATA_Z[15:8] | ACCEL_DATA_Z[15:8] | ACCEL_DATA_Z[15:8] | ACCEL_DATA_Z[15:8] | ACCEL_DATA_Z[15:8] |
| 24 | 36 | ACCEL_DATA_Z0 | SYNCR | ACCEL_DATA_Z[7:0] | ACCEL_DATA_Z[7:0] | ACCEL_DATA_Z[7:0] | ACCEL_DATA_Z[7:0] | ACCEL_DATA_Z[7:0] | ACCEL_DATA_Z[7:0] | ACCEL_DATA_Z[7:0] | ACCEL_DATA_Z[7:0] |
| 25 | 37 | GYRO_DATA_X1 | SYNCR | GYRO_DATA_X[15:8] | GYRO_DATA_X[15:8] | GYRO_DATA_X[15:8] | GYRO_DATA_X[15:8] | GYRO_DATA_X[15:8] | GYRO_DATA_X[15:8] | GYRO_DATA_X[15:8] | GYRO_DATA_X[15:8] |
| 26 | 38 | GYRO_DATA_X0 | SYNCR | GYRO_DATA_X[7:0] | GYRO_DATA_X[7:0] | GYRO_DATA_X[7:0] | GYRO_DATA_X[7:0] | GYRO_DATA_X[7:0] | GYRO_DATA_X[7:0] | GYRO_DATA_X[7:0] | GYRO_DATA_X[7:0] |
| 27 | 39 | GYRO_DATA_Y1 | SYNCR | GYRO_DATA_Y[15:8] | GYRO_DATA_Y[15:8] | GYRO_DATA_Y[15:8] | GYRO_DATA_Y[15:8] | GYRO_DATA_Y[15:8] | GYRO_DATA_Y[15:8] | GYRO_DATA_Y[15:8] | GYRO_DATA_Y[15:8] |
| 28 | 40 | GYRO_DATA_Y0 | SYNCR | GYRO_DATA_Y[7:0] | GYRO_DATA_Y[7:0] | GYRO_DATA_Y[7:0] | GYRO_DATA_Y[7:0] | GYRO_DATA_Y[7:0] | GYRO_DATA_Y[7:0] | GYRO_DATA_Y[7:0] | GYRO_DATA_Y[7:0] |
| 29 | 41 | GYRO_DATA_Z1 | SYNCR | GYRO_DATA_Z[15:8] | GYRO_DATA_Z[15:8] | GYRO_DATA_Z[15:8] | GYRO_DATA_Z[15:8] | GYRO_DATA_Z[15:8] | GYRO_DATA_Z[15:8] | GYRO_DATA_Z[15:8] | GYRO_DATA_Z[15:8] |
| 2A | 42 | GYRO_DATA_Z0 | SYNCR | GYRO_DATA_Z[7:0] | GYRO_DATA_Z[7:0] | GYRO_DATA_Z[7:0] | GYRO_DATA_Z[7:0] | GYRO_DATA_Z[7:0] | GYRO_DATA_Z[7:0] | GYRO_DATA_Z[7:0] | GYRO_DATA_Z[7:0] |
| 2B | 43 | TMST_FSYNCH | SYNCR | TMST_FSYNC_DATA[15:8] | TMST_FSYNC_DATA[15:8] | TMST_FSYNC_DATA[15:8] | TMST_FSYNC_DATA[15:8] | TMST_FSYNC_DATA[15:8] | TMST_FSYNC_DATA[15:8] | TMST_FSYNC_DATA[15:8] | TMST_FSYNC_DATA[15:8] |
| 2C | 44 | TMST_FSYNCL | SYNCR | TMST_FSYNC_DATA[7:0] | TMST_FSYNC_DATA[7:0] | TMST_FSYNC_DATA[7:0] | TMST_FSYNC_DATA[7:0] | TMST_FSYNC_DATA[7:0] | TMST_FSYNC_DATA[7:0] | TMST_FSYNC_DATA[7:0] | TMST_FSYNC_DATA[7:0] |
| 2D | 45 | INT_STATUS | R/C | - | UI_FSYNC_INT | PLL_RDY_INT | RESET_DONE_INT | DATA_RDY_INT | FIFO_THS_INT | FIFO_FULL_INT | AGC_RDY_INT |
| 2E | 46 | FIFO_COUNTH | R | FIFO_COUNT[15:8] | FIFO_COUNT[15:8] | FIFO_COUNT[15:8] | FIFO_COUNT[15:8] | FIFO_COUNT[15:8] | FIFO_COUNT[15:8] | FIFO_COUNT[15:8] | FIFO_COUNT[15:8] |
| 2F | 47 | FIFO_COUNTL | R | FIFO_COUNT[7:0] | FIFO_COUNT[7:0] | FIFO_COUNT[7:0] | FIFO_COUNT[7:0] | FIFO_COUNT[7:0] | FIFO_COUNT[7:0] | FIFO_COUNT[7:0] | FIFO_COUNT[7:0] |
| 30 | 48 | FIFO_DATA | R | FIFO_DATA | FIFO_DATA | FIFO_DATA | FIFO_DATA | FIFO_DATA | FIFO_DATA | FIFO_DATA | FIFO_DATA |
| 31 | 49 | APEX_DATA0 | SYNCR | STEP_CNT[7:0] | STEP_CNT[7:0] | STEP_CNT[7:0] | STEP_CNT[7:0] | STEP_CNT[7:0] | STEP_CNT[7:0] | STEP_CNT[7:0] | STEP_CNT[7:0] |
| 32 | 50 | APEX_DATA1 | SYNCR | STEP_CNT[15:8] | STEP_CNT[15:8] | STEP_CNT[15:8] | STEP_CNT[15:8] | STEP_CNT[15:8] | STEP_CNT[15:8] | STEP_CNT[15:8] | STEP_CNT[15:8] |
| 33 | 51 | APEX_DATA2 | R | STEP_CADENCE | STEP_CADENCE | STEP_CADENCE | STEP_CADENCE | STEP_CADENCE | STEP_CADENCE | STEP_CADENCE | STEP_CADENCE |

**Таблица 1. Регистровая карта**

---

Спецификация гироскопа из официального даташита модуля:

*Typical Operating Circuit of section 4.2, VDD = 1.8 V, VDDIO = 1.8 V, TA=25°C, unless otherwise noted.*

| PARAMETER | CONDITIONS | MIN | TYP | MAX | UNITS | NOTES |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **GYROSCOPE SENSITIVITY** | | | | | | |
| Full-Scale Range | GYRO_FS_SEL = 0 | | ±2000 | | º/s | 2 |
| | GYRO_FS_SEL = 1 | | ±1000 | | º/s | 2 |
| | GYRO_FS_SEL = 2 | | ±500 | | º/s | 2 |
| | GYRO_FS_SEL = 3 | | ±250 | | º/s | 2 |
| | GYRO_FS_SEL = 4 | | ±125 | | º/s | 2 |
| | GYRO_FS_SEL = 5 | | ±62.5 | | º/s | 2 |
| | GYRO_FS_SEL = 6 | | ±31.25 | | º/s | 2 |
| | GYRO_FS_SEL = 7 | | ±15.625 | | º/s | 2 |
| Gyroscope ADC Word Length | Output in two’s complement format | | 16 | | bits | 2, 5 |
| Sensitivity Scale Factor | GYRO_FS_SEL = 0 | | 16.4 | | LSB/(º/s) | 2 |
| | GYRO_FS_SEL = 1 | | 32.8 | | LSB/(º/s) | 2 |
| | GYRO_FS_SEL = 2 | | 65.5 | | LSB/(º/s) | 2 |
| | GYRO_FS_SEL = 3 | | 131 | | LSB/(º/s) | 2 |
| | GYRO_FS_SEL = 4 | | 262 | | LSB/(º/s) | 2 |
| | GYRO_FS_SEL = 5 | | 524.3 | | LSB/(º/s) | 2 |
| | GYRO_FS_SEL = 6 | | 1048.6 | | LSB/(º/s) | 2 |
| | GYRO_FS_SEL = 7 | | 2097.2 | | LSB/(º/s) | 2 |
| Sensitivity Scale Factor Initial Tolerance | Component and Board-level, 25°C | | ±0.5 | | % | 1 |
| Sensitivity Scale Factor Variation Over Temperature | 0°C to +70°C | | ±0.005 | | %/°C | 3 |
| Nonlinearity | Best fit straight line; 25°C | | ±0.1 | | % | 3 |
| Cross-Axis Sensitivity | Board-level | | ±1.25 | | % | 3 |
| **ZERO-RATE OUTPUT (ZRO)** | | | | | | |
| Initial ZRO Tolerance | Board-level, 25°C | | ±0.5 | | º/s | 3 |
| ZRO Variation vs. Temperature | 0°C to +70°C | | ±0.005 | | º/s/°C | 3 |
| **OTHER PARAMETERS** | | | | | | |
| Rate Noise Spectral Density | @ 10 Hz | | 0.0028 | | º/s / √Hz | 1 |
| Total RMS Noise | Bandwidth = 100 Hz | | 0.028 | | º/s-rms | 4 |
| Gyroscope Mechanical Frequencies | | 25 | 27 | 29 | KHz | 1 |
| Low Pass Filter Response | ODR < 1kHz | 5 | | 500 | Hz | 2 |
| | ODR ≥ 1kHz | 42 | | 3979 | Hz | 2 |
| Gyroscope Start-Up Time | Time from gyro enable to gyro drive ready | | 30 | | ms | 3 |
| Output Data Rate | | 12.5 | | 32000 | Hz | 2 |

**Таблица 2. Спецификация гироскопа**

---

Спецификация акселерометра из официального даташита модуля:

*Typical Operating Circuit of section 4.2, VDD = 1.8 V, VDDIO = 1.8 V, TA=25°C, unless otherwise noted.*

| PARAMETER | CONDITIONS | MIN | TYP | MAX | UNITS | NOTES |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **ACCELEROMETER SENSITIVITY** | | | | | | |
| Full-Scale Range | ACCEL_FS_SEL = 0 | | ±16 | | g | 2 |
| | ACCEL_FS_SEL = 1 | | ±8 | | g | 2 |
| | ACCEL_FS_SEL = 2 | | ±4 | | g | 2 |
| | ACCEL_FS_SEL = 3 | | ±2 | | g | 2 |
| ADC Word Length | Output in two’s complement format | | 16 | | bits | 2, 5 |
| Sensitivity Scale Factor | ACCEL_FS_SEL = 0 | | 2,048 | | LSB/g | 2 |
| | ACCEL_FS_SEL = 1 | | 4,096 | | LSB/g | 2 |
| | ACCEL_FS_SEL = 2 | | 8,192 | | LSB/g | 2 |
| | ACCEL_FS_SEL = 3 | | 16,384 | | LSB/g | 2 |
| Sensitivity Scale Factor Initial Tolerance | Component and Board-level, 25°C | | ±0.5 | | % | 1 |
| Sensitivity Change vs. Temperature | -40°C to +85°C | | ±0.005 | | %/°C | 3 |
| Nonlinearity | Best Fit Straight Line, ±2g | | ±0.1 | | % | 3 |
| Cross-Axis Sensitivity | Board-level | | ±1 | | % | 3 |
| **ZERO-G OUTPUT** | | | | | | |
| Initial Tolerance | Board-level, all axes | | ±20 | | mg | 3 |
| Zero-G Level Change vs. Temperature | -40°C to +85°C | | ±0.15 | | mg/°C | 3 |
| **OTHER PARAMETERS** | | | | | | |
| Power Spectral Density | @ 10 Hz (X and Y-axis) | | 65 | | μg/√Hz | 1 |
| | @ 10 Hz (Z-axis) | | 70 | | μg/√Hz | 1 |
| RMS Noise | Bandwidth = 100 Hz (X and Y-axis) | | 0.65 | | mg-rms | 4 |
| | Bandwidth = 100 Hz (Z-axis) | | 0.70 | | mg-rms | 4 |
| Low-Pass Filter Response | ODR < 1kHz | 5 | | 500 | Hz | 2 |
| | ODR ≥ 1kHz | 42 | | 3979 | Hz | 2 |
| Accelerometer Startup Time | From sleep mode to valid data | | 10 | | ms | 3 |
| Output Data Rate | | 1.5625 | | 32000 | Hz | 2 |

**Таблица 3. Спецификация акселерометра**

---

## Назначение параметров в математической модели погрешностей

В цифровом двойнике приведенные паспортные параметры формируют модель виртуальных датчиков, преобразующую идеальную кинематику в реальные зашумленные сигналы:

* **Смещение нуля гироскопа ($b_{g0}=\pm 0.5^\circ/\text{с}$):** задает постоянный систематический дрейф ориентации дрона. Ошибка угла нарастает линейно ($\Delta \theta \approx b_{g0} \cdot t$), что из-за проекции вектора силы тяжести $g$ приводит к кубическому накоплению ошибки плановых координат ($\delta r \sim \frac{1}{6} g b_{g0} t^3$). 
* **Спектральная плотность шума гироскопа ($\text{ND}_g=0.0028^\circ/\text{с}/\sqrt{\text{Гц}}$):** определяет среднеквадратичное отклонение ($\sigma_g=0.028^\circ/\text{с}$ в полосе 100 Гц) для дискретного генератора гауссовского белого шума $\mathcal{N}(0, \sigma_g^2)$. Ошибка ориентации от шума растет пропорционально $\sqrt{t}$  
* **Температурный градиент ($K_{gT}=\pm 0.005^\circ/\text{с}/^\circ\text{C}$):** моделирует динамическое смещение нуля при нагреве электроники в полете по формуле $\Delta b_{g}(T)=K_{gT} \cdot \Delta T$. 
* **Погрешность масштабного коэффициента ($s=\pm 0.5\%$):** проявляется исключительно при маневрах облета, искажая амплитуду угловой скорости ($\omega_{\text{изм}}=\omega_{\text{ист}}(1 \pm 0.005)$), что создает ошибки недоворота/переворота на виражах. 
* **Перекрестная чувствительность ($m=\pm 1.25\%$):** задает недиагональные связи каналов, моделируя паразитную утечку угловой скорости одного канала (например, крена) в смежные оси тангажа и курса. 
* **Смещение нуля акселерометра ($b_{a0}=\pm 20\text{ мг} \approx \pm 0.196\text{ м/с}^2$):** при двойном интегрировании дает квадратичный рост позиционной погрешности ($\Delta S \approx \frac{1}{2} b_{a0} t^2$), определяя предел автономности БИНС без внешней коррекции.
* **Предел измерений ($\omega=\pm 2000^\circ/\text{с}$):** максимальная угловая скорость, которую может измерить датчик, при превышении значения датчик обрежет верхушку угловой скорости, что отразится на его позиционировании.

Данные параметры будут использоваться при моделировании погрешности реального полёта БПЛА.