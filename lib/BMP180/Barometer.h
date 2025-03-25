/****************************************************************
*   Standart deniz seviyesi basıncı 101325 Pa'dır.              *
*                                                               *
*   Standart deniz seviyesi basıncı (p) :1013.25 Mb             *
*                                                               *
*   Bu kütüphane BMP sensörü için kalıtımlı bir kütüphanedir.   *
*                                                               *
*****************************************************************/

#ifndef BAROMETER_H
#define BAROMETER_H

#include <Adafruit_BMP085.h>
#include <Arduino.h>

class Barometer{
public:
    // Yapıcı: filtre penceresi boyutunu belirleyebilirsin (varsayılan 10)
    Barometer(uint8_t filterSize = 10);
    ~Barometer();

    // BMP sensörünü başlatır, başarılıysa true döner
    bool begin();

    // Ham altimetre değerini döner (referans basıncı parametresi ile)
    float readMyAltitude();
    // aslında gerek yok ama ne olur ne olmaz koydum
    float readMyPressure();

    // Drone için stabil, filtrelenmiş yükseklik değerini döner
    float getMyAltitude();

    // Basınç değerini döner
    float getMyPressure();

    // Ev noktası (referans) ile karşılaştırarak yükseklik hesaplar
    float getAltitudeFromHomePoint(float pressure);

    // It calibrates the barometer offset (sea level = 101325)
    float calibrateGyro();


private:
    // Dahili BMP180 sensör nesnesi
    Adafruit_BMP085 bmp;

    // Hareketli ortalama filtre için dahili değişkenler
    uint8_t _filterSize;
    float* _buffer;
    uint8_t _bufferIndex;
    
    // İç filtre metodu (hareketli ortalama)
    float movingAverage(float newValue);
};

#endif

/*
⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⣀⣀⣀⣀⣀⣀⣀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⣤⣶⠿⠿⠿⠿⠛⠛⠛⠛⠛⠛⠛⠛⠛⠻⠿⠿⠿⠿⠿⠷⠶⣶⣶⣦⣤⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⣼⡟⠛⠷⣦⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⠛⠿⣶⣤⣀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⣿⠀⠀⠀⠀⠈⠙⠻⢶⣦⣤⣤⣤⣤⡤⠤⠤⠤⠤⠤⠤⢤⣤⣤⣤⣤⣀⣀⣀⣀⣀⡀⠉⠛⢿⣶⣄⠀⠀⠀⠀
⠀⠀⢰⡿⠀⠀⠀⣄⠀⠀⠀⠀⢸⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠉⠉⠛⠛⢿⣷⠀⠀⠀
⠀⠀⢸⡇⠀⣀⠀⠀⠀⠛⠀⠀⣸⡇⠀⣰⠚⠉⠉⠉⠉⠉⠉⠉⠉⠉⠉⠛⠛⠓⠒⠒⠲⠦⠤⠤⣄⡀⠀⠀⣿⠀⠀⠀
⠀⠀⣼⡇⠀⠀⠀⠛⠀⣤⠀⠀⣿⠁⢠⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢷⠀⠀⣿⠀⠀⠀
⠀⠀⣿⠁⠀⠰⠆⢀⣄⠀⠀⢠⣿⠀⢸⠀⠀⠀⠀⠀⢰⣾⡆⠀⠀⠀⠀⠀⠀⠀⢠⣤⡄⠀⠀⠀⠀⢸⠀⢰⣿⠀⠀⠀
⠀⢀⣿⠀⠀⠀⠀⠀⠁⠀⠀⢸⡇⠀⡜⠀⠀⠀⠀⠀⠀⠉⠀⠀⢀⣀⣀⣀⡀⠀⠈⠛⠃⠀⠀⠀⠀⡸⠀⢸⡿⠀⠀⠀
⠀⢸⣿⠀⢠⣤⣀⠀⠀⠀⠀⣼⠇⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⠿⢿⠿⠃⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⣸⡇⠀⠀⠀
⠀⢸⡿⠀⢸⣏⣿⢷⡆⠀⠀⣿⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⣿⠇⠀⠀⠀
⠀⣼⡇⠀⠈⠻⠻⣾⠃⠀⢀⣿⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠃⠀⣿⠀⠀⠀⠀
⠀⣿⡇⠀⢸⣷⣦⣀⠀⠀⢸⡇⠀⠠⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⠀⢸⡏⠀⠀⠀⠀
⠀⣿⡇⠀⠀⣼⣿⡿⠀⠀⢸⠇⠀⠀⠧⣤⣄⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡇⠀⣼⠇⠀⠀⠀⠀
⢸⣿⠃⠰⢿⣿⣿⡆⠀⠀⣼⠀⡴⠛⣶⡄⠀⠀⠉⠉⠉⠉⠉⠉⠉⠉⠙⠛⠛⠛⠛⠛⠋⠉⠉⠉⠀⠀⣿⠀⠀⠀⠀⠀
⢸⣿⠀⠀⠀⠈⠉⠁⠀⠀⣿⣼⠃⠀⣸⣷⣶⣶⣶⣶⣶⣶⣶⣶⣤⣤⡄⠀⠀⠀⠀⢀⣤⡀⠀⠀⠀⢰⡏⠀⠀⠀⠀⠀
⢸⣿⠀⣴⣾⣿⣶⣄⠀⢀⣿⠇⠀⣼⠁⠀⠈⠉⠉⠉⠉⠉⠉⠉⠉⠉⠁⠀⠀⠀⠀⠈⠛⠁⠀⠀⠀⣸⣷⣄⡀⠀⠀⠀
⣸⣿⠘⣿⡟⠙⣿⡿⠀⣸⠏⢀⣼⠃⠀⢠⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡿⠈⠉⠛⢶⣄⠀
⣿⣿⠀⠙⢳⣄⠘⠓⠚⠁⣠⡾⠁⠀⡖⠚⠀⠓⠄⠀⠀⠀⠀⠀⠀⢀⡾⠋⢆⠀⢀⡤⠄⡀⠀⠀⢠⣿⣶⣶⡄⠀⠙⣷
⣿⡇⠀⠀⠀⠙⠳⠶⠶⢾⡏⠀⠀⠀⠑⣆⠀⠒⠂⠀⠀⠀⠀⠀⠀⠀⠉⠀⠈⠀⠸⣄⣀⠄⠀⠀⢸⣿⣿⡿⠃⢀⣴⠏
⣿⣇⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⠟⠛⠓⣄⠀⠀⠀⠀⠀⣿⣿⠟⠀⣠⠟⠁⠀
⠈⠙⠿⣦⣄⠀⠀⠀⠀⢸⡇⠀⠀⠀⣠⣤⣀⠀⠀⢀⣀⣀⠀⠀⠀⠀⣿⡀⠀⠀⡼⠀⠀⠀⠀⢠⣿⣿⣴⣠⣽⠆⠀⠀
⠀⠀⠀⠀⠉⠛⠶⣤⣄⣿⡇⠀⠀⠀⠀⠉⠀⠀⠀⠈⠉⠉⠀⠀⠀⠀⠈⠙⠓⠋⠀⠀⠀⠀⠀⢸⡟⠈⠉⠉⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠉⠛⠛⠻⠿⣶⡶⠶⢶⣶⣤⣤⣤⣤⣤⣤⣤⣤⣄⣀⣀⣀⣀⣀⣠⣤⣤⡿⠃⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⣦⡀⠻⣧⠀⠀⠀⠀⢈⡟⠀⢸⡟⠉⠉⠉⠉⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠴⠚⠋⠁⣠⡿⠀⠀⠀⠀⣸⠃⠀⣿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡇⠀⣤⡶⠟⠉⠀⠀⠀⠀⢀⡟⠀⣸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⢷⣄⣹⡇⠀⠀⠀⠀⠀⠀⣼⠃⢀⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⠛⠁⠀⠀⠀⠀⠀⢠⡟⠀⢸⣧⣤⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⣧⣀⣀⣠⡼⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
*/