# ESP32 Button Simulator (MQTT + Web UI + OTA)

Прошивка для ESP32, которая имитирует нажатия кнопок через «сухой контакт» (реле 1–4 канала). Управление — через MQTT и веб-интерфейс. Есть OTA.

## Логика нажатия

Нажатие = замыкание (реле ON) → пауза `press_ms` → размыкание (реле OFF).



## Подключение

- Кнопочная цепь подключается на контакты реле **COM** и **NO** (замыкаются на время нажатия).
- Релейный модуль должен иметь общий **GND** с ESP32.

Про ваш модуль: «перемычка стоит на VCC и активируется землёй» — это **active LOW** вход. Поэтому по умолчанию в проекте `BTN_DEFAULT_RELAY_INVERTED=1`.

## Веб

- `/` — главная (настройки, статус, тестовые кнопки Button 1..4 — по `relay_count`).
- `/update` — обновление прошивки через браузер.
- `/logs` — логи.

## MQTT (по умолчанию)

Базовый топик: `buttons/<deviceId>/`

Команды:

- `cmd/button1` — payload `PRESS` (или `1`, `ON`, `true`)
- `cmd/button2` — payload `PRESS` (или `1`, `ON`, `true`)
- `cmd/button3` — payload `PRESS` (если `relay_count>=3`)
- `cmd/button4` — payload `PRESS` (если `relay_count>=4`)
- `cmd/enabled` — `1`/`0`
- `cmd/press_ms` — число (мс), настройка длительности нажатия (также доступно из HA как Number)

Состояния:

- `state/enabled` — `1`/`0`
- `state/relay1` — `ON`/`OFF`
- `state/relay2` — `ON`/`OFF`
- `state/relay3` — `ON`/`OFF` (если `relay_count>=3`)
- `state/relay4` — `ON`/`OFF` (если `relay_count>=4`)
- `state/press_ms` — число (мс)
- `state/relay_count` — число (1..4)
- `state/reed` — `OPEN`/`CLOSED` (только если геркон активирован в веб)
- `status/online` — `1`/`0`

Примечание: если в MQTT/HA всё ещё виден `gap_ms`, это старый retained от предыдущих версий. Текущая прошивка очищает retained `state/gap_ms` и HA discovery `gap_ms` при подключении к MQTT.

### Геркон (пассивный)

Геркон — отдельный датчик, не связан с реле.

- В веб можно включить/выключить «Activate reed sensor», выбрать GPIO и тип контакта `NO/NC`.
- В прошивке используется `INPUT_PULLUP`: контакт замыкается на GND.
- Если геркон **выключен**, прошивка не публикует его состояние в MQTT (и очищает retained `state/reed`).

## PlatformIO

Сборка/прошивка аналогично проекту увлажнителя.
