/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define FC_TARGET_MCU     RP2350A
#define BOARD_NAME        PICO2_2350A
#define MANUFACTURER_ID   RASP

// --------------------------------------------------------------------
// PIN MAP (from your table; GPIO N == PA N)
//
// UART0 (radio):   TX=GPIO0  RX=GPIO1
// PWM outputs:     OUT0..OUT5 = GPIO2..GPIO7 (motor/servo1..6)
// UART1 (GPS):     TX=GPIO8  RX=GPIO9
// I2C1 (gyro alt): SDA=GPIO10 SCL=GPIO11
// SDCard (SPI1):   MISO=GPIO12 CS=GPIO13 SCK=GPIO14 MOSI=GPIO15
//
// IMU (SPI0):      MISO=GPIO16 CS=GPIO17 SCK=GPIO18 MOSI=GPIO19
// I2C0 (sensors):  SDA=GPIO20 SCL=GPIO21 (baro/mag etc)
// IMU INT:         GPIO22
//
// ADC: VBAT=GPIO28 (A2)  CURR=GPIO27 (A1)
// --------------------------------------------------------------------


// ------------------
// Serial (External)
// ------------------

// UART0 - RC Radio (CRSF/ELRS etc)
#define UART0_TX_PIN         PA0
#define UART0_RX_PIN         PA1

// UART1 - GPS
#define UART1_TX_PIN         PA8
#define UART1_RX_PIN         PA9


// ------------------
// Motors / Servos
// ------------------

// pin_out0..pin_out5 -> motor/servo 1..6
#define MOTOR1_PIN           PA2


// ------------------
// IMU (SPI0 on GPIO16/18/19 + CS17, INT22)
// ------------------

#define USE_GYRO
#define USE_ACC

// Compile BOTH so clones / variants still detect
#define USE_GYRO_SPI_MPU9250
#define USE_ACC_SPI_MPU9250
#define USE_GYRO_SPI_MPU6500
#define USE_ACC_SPI_MPU6500

#define USE_SPI_DEVICE_0
#define SPI0_SCK_PIN         PA18
#define SPI0_SDI_PIN         PA16  // MISO
#define SPI0_SDO_PIN         PA19  // MOSI

#define GYRO_1_CS_PIN        PA17
#define GYRO_1_EXTI_PIN      PA22
#define GYRO_1_SPI_INSTANCE  SPI0

// Set this to match your physical board orientation.
// If unsure, start with CW0 and adjust after first gyro check in Configurator.
#define GYRO_1_ALIGN         CW0_DEG


// ------------------
// I2C buses
// ------------------

// I2C0: for barometer/magnetometer/etc (your table: GPIO20/21)
#define I2C0_SDA_PIN         PA20
#define I2C0_SCL_PIN         PA21

// I2C1: listed as "I2C gyro SDA/SCL" in your table (GPIO10/11)
// (Optional - keep enabled so you can hang other sensors here if desired)
#define I2C1_SDA_PIN         PA10
#define I2C1_SCL_PIN         PA11


// ------------------
// ADC (Battery / Current)
// ------------------

#define USE_ADC
#define ADC_INSTANCE         ADC1

// VBAT divider -> (A2) GPIO28
#define ADC_VBAT_PIN         PA28
#define DEFAULT_VOLTAGE_METER_SOURCE   VOLTAGE_METER_ADC

// NOTE: scale depends on your resistor divider ratio.
// This is a placeholder; set it to your actual divider calibration value.
#define DEFAULT_VOLTAGE_METER_SCALE    110

// Current sensor -> (A1) GPIO27
#define ADC_CURR_PIN         PA27
#define DEFAULT_CURRENT_METER_SOURCE   CURRENT_METER_ADC

// ------------------
// Optional external Baro on I2C0
// ------------------

// Leave these enabled only if you actually wire supported sensors to I2C0.
// If you don't use them, you can comment them out to reduce feature footprint.
#define USE_BARO
#define USE_BARO_MS5611
#define USE_BARO_BMP280
#define USE_BARO_BMP388
#define USE_BARO_LPS
#define USE_BARO_QMP6988
#define USE_BARO_DPS310
#define USE_BARO_BMP085
#define USE_BARO_2SMBP_02B
#define USE_BARO_LPS22DF
#define BARO_I2C_INSTANCE    I2CDEV_0

// ------------------
// Magnetometer (HMC5883L on I2C0)
// ------------------
#define USE_MAG_QMC5883
#define USE_MAG_HMC5883

// If your target config supports explicit bus selection macros, keep it on the same I2C bus as your baro:
#define MAG_I2C_DEVICE       I2CDEV_0
// Optional (only if your tree expects it / you want to force address):
// #define MAG_I2C_ADDRESS    0x1E
