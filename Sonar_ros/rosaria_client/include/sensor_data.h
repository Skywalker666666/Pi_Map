//============================================================================
// Copyright   : PerceptIn
//============================================================================
#ifndef  __sensor_data_H__
#define  __sensor_data_H__
#include <vector>
namespace PIRoBot {

// ScanData can be laser, sonar or infra-red light, their data can be abstracted to a scan line.
// (sx, sy) is the sensor position relative to robot center.
// th is theta, the heading of the sensor relative to robot center.
// len is the length of current scan line.
class ScanData {
  public:
    double sx;
    double sy;
    double th;
    double len;
    ScanData(): sx(0), sy(0), th(0), len(0) {};
    ScanData(double _sx, double _sy, double _th, double _len) : sx(_sx), sy(_sy), th(_th), len(_len) {};
};

class SensorScan : public std::vector<ScanData> {

};


}
#endif   /* ----- #ifndef __sensor_data_H__  ----- */

