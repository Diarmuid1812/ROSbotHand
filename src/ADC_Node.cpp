#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <Rasia_Prototyp/zeros.h>
#include <std_msgs/String.h>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <linux/i2c.h>  
#include <linux/module.h>
#include <i2c/smbus.h>
}

int file_i2c;

const uint8_t i2caddress = 0x04;
const uint8_t startVoltRegAddr = 0X20;
const int nchannels = 8;

const size_t samples = 1500;
double threshhold;

double convert(uint16_t val);
bool readVoltage(std::vector<uint16_t>& v);
bool moveDetected(const std::vector<double>& vol, double threshhold_val);
std::vector<uint16_t> get_zero_crossing(const std::vector<std::vector<uint16_t>>& data);


int main(int argc, char** argv)
{
    uint16_t channels[nchannels] = {0,0,0,0,0,0,0,0};

    char *filename = (char*)"/dev/i2c-1";
    if ((file_i2c = open(filename, O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check errno to see what went wrong
        std::cout << file_i2c << std::endl;
        std::cout << "Failed to open the i2c bus" << std::endl;
        return EXIT_FAILURE;
    }

    if (ioctl(file_i2c, I2C_SLAVE, i2caddress) < 0)
    {
        std::cout << "Failed to acquire bus access and/or talk to slave.\n" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Start ADC node " << std::endl;
    ros::init(argc, argv, "ADC_NODE");
    ros::NodeHandle n;
    ros::Publisher adc_publisher =
        n.advertise<std_msgs::String>("ADCtalker", 100);	// nazwa talkera i wielkosc bufora wiadomosci
	ros::Rate loop_rate(10); // 10 msgs/sec
	
    ros::param::get("/ADC_node/threshold", threshhold);
    std::vector<uint16_t> measurements(nchannels);
    bool readingOk = true;
    std::vector<std::vector<uint16_t>> data(samples);
    std::vector<std::vector<uint16_t>> temp(samples);
    std::vector<double> converteed_val(nchannels);
    clock_t t, start, stop;

    double timeInSec, clocks, waitTime;
    struct timespec tim, tim2;
    tim.tv_nsec = 0;
    double timestamp = 0.001;
    Rasia_Prototyp::zeros msg;
    while (ros::ok() and readingOk)
    {
        std::cout << "===========START MEASUREMENT===================" << std::endl;
        t = clock();
        start = t;
        readingOk = readVoltage(measurements);
        std_msgs::String msg;
        std::stringstream ss;
        
        for (int i=0;i < nchannels;++i)
        {
            converteed_val[i] = convert(measurements[i]);
            //std::cout << converteed_val[i] << " | ";
        }
        //std::cout << std::endl;
        
        if (moveDetected(converteed_val, threshhold))
        {
            data[0] = measurements;
            for (int i = 1; i < samples; ++i)
            {
                clocks = clock() - t;

                tim.tv_sec = timestamp - (clocks/CLOCKS_PER_SEC);
                nanosleep(&tim, &tim2);
                t = clock();
                readingOk = readVoltage(measurements);
                if (not readingOk)
                {
                    std::cout << "ERROR while reading" << std::endl;
                    break;
                }
                data[i] = measurements;

            }
            std::cout << "===========STOP MEASUREMENT===================" << std::endl;
            //msg.samples = get_zero_crossing(data);
            

            /******************** konwertowanie danych ******************
            * zapisuje calosc do wielkiego sringa
            * tworzac macierz [8][1500] (tak uklada dane, ale w jednym wierszu zeby pozniej latwiej bylo sparsowac)
            * poszczegolne dane oddzielone sa spacja
            * na koncu kazdego wiersza (po 8 danych) jest ;
            * przyklad: "1 2 3 4 5 6 7 8; 1 2 3 4 5 6 7 8; ... ; 1 2 3 4 5 6 7 8"
            * ***********************************************************/

            for(int i = 0; i < samples; i++)	// 1500
            {
				for(int j = 0; j < nchannels; j++)	// 8
				{
					temp[i][j] = convert(data[i][j]);	//dla bezpieczenstwa zapisywane w oddzielnej tablicy

					ss << std::to_string(temp[i][j]);
					if(j != nchannels - 1) ss << ' '; //po ostatniej probce w wierszu nie dodaje spacji
				}

				if(i != samples - 1) ss << ';'; //po ostatnim wierszu nie dodaje srednika
			}
            msg.data = ss.str();
            adc_publisher.publish(msg);
            //ROS_INFO("%s", msg.data.c_str()); // wypisuje wiadomosc w terminalu (bedzie strasznie dluga wiec moze lepiej nie)
            ros::spinOnce(); //do odbierania pakietow - niepotrzebne ale dobra praktyka
			loop_rate.sleep();
        }
        else
        {
            std::cout << "===========NO MOVE===================" << std::endl;
        }
        sleep(10);
    }

    close(file_i2c);
}

//==========================================================================
//==========================================================================
//==========================FUNCTIONS=======================================
//==========================================================================
//==========================================================================

double convert(uint16_t val)
{
    return ((static_cast<double>(val) * (10./ 3.3)) - 5000.)/1000.;
}

bool moveDetected(const std::vector<double>& vol, double threshhold_val)
{
    for (auto &channel : vol)
    {
        if (std::abs(channel) > threshhold_val)
        {
            return true;
        }
    }
    return false;
}

bool readVoltage(std::vector<uint16_t>& v)
{
    uint16_t res;
    for (int i = 0; i < nchannels; ++i)
    {
        res = i2c_smbus_read_word_data(file_i2c, startVoltRegAddr+i);
        if (res < 0)
        {
            return false;
        }
        else
        {
            v[i] = res;
        }
    }
    return true;
}

std::vector<uint16_t> get_zero_crossing(const std::vector<std::vector<uint16_t>>& data)
{
    std::vector<uint16_t> channels = {0,0,0,0,0,0,0,0};
    for (int i = 1; i< samples; ++i)
    {
        for (int j = 0; j<nchannels; ++j)
        {
            if (convert(data[i][j]) * convert(data[i - 1][j]) <= 0)
            {
                channels[j] += 1;
            }

        }
    }
    return channels;
}
