# Описание пакета gs_camera

## Описание:
Данный пакет предоставляет инструменты для работы с камерой

## Состав пакета:
1. Ноды:
    * web_viewer_node.py
2. Файлы запуска:
    * camera.launch - запускает камеру, и веб сервер просмотра изображение
    * photo.launch - запускает камеру и ноду съемки
3. Классы:
    * RunCam

## Необходимые пакеты:
1. Python:
    * OpenCV
    * Flask
    * crccheck
    * PySerial
2. ROS:
    * cv_camera
    * cv_bridge
    * sensor_msgs

## Описание нод:

### 1. web_viewer_node.py
Нода веб сервера для просмотра изображения с камеры в браузере
Запускает веб сервер на используя выбранный интерфейс и порт

#### Параметры:
* image_topic - топик потока с камеры (тип топика: sensor_msgs/Image)
* interface - интерфес (eth0 - Ethernet, wlan0 - Wi-fi ), по умолчанию wlan0
* port - порт, по умолчанию 8088

#### Подписки:
* топик указанный в параметре image_topic

#### Сервисы
* geoscan/make_photo (sts_srvs/Empty)

### 2. photo_node.py
Нода для съемки и сохранения изображение с EXIF данными

#### Параметры:
* image_topic - топик потока с камеры (тип топика: sensor_msgs/Image)
* folder_path - путь до папки с изображениями

#### Подписки:
* топик указанный в параметре image_topic
* geoscan/navigation/global/position

#### Сервисы:
* geoscan/navigation/get_system

## Описание классов:

### 1. RunCam
Класс взаимодействия с RunCam Hybrid

#### Инициализация:
Без параметров

#### Поля:
* _resolution - list of str
* _tv_mode - list of str
* _serial - serial.Serial

#### Методы:
* change_mode - изменяет режим камеры
* power_button - имитирует нажатие кнопки power
* close - закрывает сооединение с камерой, необходимо выполнить в конце программы
* __read_data - считывает данные, приходящие от камеры
* get_tv_mode - возвращает настройку Tv Mode
* get_resolutin - возвращает разрещение съемки
* get_sd_capacity - возвращает словарь, ключ free - кол-во свободной памяти, ключ max - кол-во памяти на SD карте
* get_remain_recording_time - возвращает оставщееся время съемки видио
* get_camera_time - возвращает время камеры
* __get_settings(settings_id, chunk) - возвращает значение, выбранной настройки, settings_id - ID настройки, chunk - номер раздела памяти
* set_resolution(resolution="4K@30FPS") - устанавливает разрещение съемки, resolution - разщение съемки (доступны следующие варианты: "4K@30FPS", "2.7K@60FPS", "2.7K@50FPS", "1080@120FPS", "1080@60FPS", "1080@50FPS", "1080@30FPS")
* set_camera_time(day, month, year) - устанавливает время камеры, day - день, month - месяц, year - год, если вызвать функцию без параметров установит текущую дату RPi
* set_tv_mode(mode="NTSC") - устанавливает Tv Mode, mode - название режима( доступны следующие варианты: "NTSC", "PAL" )
* __set_settings(settings_id, body) - установить, выбранную настройку, settings_id - ID настройки, body - тело пакета