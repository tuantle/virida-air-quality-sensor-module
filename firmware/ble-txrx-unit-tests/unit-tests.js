/**
 * Copyright (c) 2017-present, Virida, Inc. All rights reserved.
 *
 * Licensed TBA
 *
 *------------------------------------------------------------------------
 *
 * @description - Virida sensor module bluetooth Rx/TX unit test runner.
 *
 * @author Tuan Le (tuan.t.lei@gmail.com)
 *
 *------------------------------------------------------------------------
 **/

/* load babel */
/* eslint quotes: 0 */
require(`babel-register`)({
    comments: false,
    presets: [
        "node7"
    ],
    plugins: [
        "transform-strict-mode"
    ]
});

const moment = require(`moment`);
const noble = require(`noble`);

const VIRIDA_NAME = `VIRIDA`;
const VIRIDA_SERVICE_ID = `6e400001b5a3f393e0a9e50e24dcca9e`;
const VIRIDA_TX_CHARACTERISTIC_ID = `6e400002b5a3f393e0a9e50e24dcca9e`;
const VIRIDA_RX_CHARACTERISTIC_ID = `6e400003b5a3f393e0a9e50e24dcca9e`;

noble.on(`stateChange`, (state) => {
    if (state === `poweredOn`) {
        noble.startScanning([
            VIRIDA_NAME
        ], false);
    } else {
        noble.stopScanning();
    }
});

noble.on(`discover`, (peripheral) => {
    noble.stopScanning();
    peripheral.connect((peripheralError) => {
        if (peripheralError) {
            console.log(peripheralError);
        } else {
            peripheral.discoverServices([
                VIRIDA_SERVICE_ID
            ], (serviceError, services) => {
                if (serviceError) {
                    console.log(serviceError);
                } else {
                    if (services.length) {
                        const [
                            service
                        ] = services;
                        service.discoverCharacteristics([
                            VIRIDA_TX_CHARACTERISTIC_ID,
                            VIRIDA_RX_CHARACTERISTIC_ID
                        ], (characteristicError, characteristics) => {
                            if (characteristicError) {
                                console.log(characteristicError);
                            } else {
                                if (characteristics.length) {
                                    const [
                                        txCharacteristic,
                                        rxCharacteristic
                                    ] = characteristics;
                                    rxCharacteristic.subscribe((rxSubcribeError) => {
                                        console.log(rxSubcribeError);
                                    });

                                    let xpkgLen = 0;

                                    rxCharacteristic.on(`data`, (data) => {
                                        if (data.byteLength === 2) {
                                            if (data.readUInt8(0) === 0xC0) {
                                                xpkgLen = data.readUInt8(1);
                                            }
                                        }
                                        if (xpkgLen === 14 && data.byteLength === xpkgLen) {
                                            const temp = data.readUInt16BE(0);
                                            const humidity = data.readUInt16BE(2);
                                            const aqi = data.readUInt16BE(4);
                                            const dustDensity = data.readUInt16BE(6);
                                            const coPPM = data.readUInt16BE(8);
                                            const co2PPM = data.readUInt32BE(10);
                                            console.log(moment().format('LTS'));
                                            console.log(`Package len: ${xpkgLen}`);
                                            console.log(`\n`);
                                            console.log(`Temp: ${temp / 100} C`);
                                            console.log(`Humidity: ${humidity / 100} %`);
                                            console.log(`AQI: ${aqi}`);
                                            console.log(`Dust Density: ${dustDensity / 100} ug/m^3`);
                                            console.log(`CO: ${coPPM / 100} ppm`);
                                            console.log(`CO2: ${co2PPM / 100} ppm`);
                                            console.log(`\n`);
                                        }
                                    });

                                    setTimeout(() => {
                                        console.log(`TEST1`);
                                        console.log(`Requesting Module Info...`);
                                        let txDataBuff = Buffer(2);
                                        let pkgLen = 0;

                                        txDataBuff.writeUInt8(0x90, 0);
                                        txDataBuff.writeUInt8(0x00, 1);

                                        txCharacteristic.write(txDataBuff, true, () => {
                                            rxCharacteristic.on(`data`, (data) => {
                                                if (pkgLen === 0) {
                                                    if (data.readUInt8(0) === 0x90) {
                                                        pkgLen = data.readUInt8(1);
                                                    }
                                                } else {
                                                    const [
                                                        rev,
                                                        skew,
                                                        firmwareVer
                                                    ] = data.toString('ascii').split(`;`);
                                                    console.log(`Package len: ${pkgLen}`);
                                                    console.log(`Module rev: ${rev}`);
                                                    console.log(`Module Skew: ${skew}`);
                                                    console.log(`Module Firmware Version: ${firmwareVer}`);
                                                    pkgLen = 0;
                                                }
                                            });
                                        });
                                    }, 1000);

                                    setTimeout(() => {
                                        console.log(`TEST2`);
                                        console.log(`Requesting Module Battery Lvl Info...`);
                                        let txDataBuff = Buffer(2);
                                        let pkgLen = 0;

                                        txDataBuff.writeUInt8(0xA0, 0);
                                        txDataBuff.writeUInt8(0x00, 1);
                                        txCharacteristic.write(txDataBuff, true, () => {
                                            rxCharacteristic.on(`data`, (data) => {
                                                if (pkgLen === 0) {
                                                    if (data.readUInt8(0) === 0xA0) {
                                                        pkgLen = data.readUInt8(1);
                                                    }
                                                } else {
                                                    const battLvl = data.readUInt8(0);
                                                    console.log(`Package len: ${pkgLen}`);
                                                    console.log(`Battery Lvl: ${battLvl} %`);
                                                    pkgLen = 0;
                                                }
                                            });
                                        });
                                    }, 5000);

                                    // setTimeout(() => {
                                    //     console.log(`TEST3`);
                                    //     console.log(`Requesting Module Whoami Signaling...`);
                                    //     let txDataBuff = Buffer(2);
                                    //
                                    //     txDataBuff.writeUInt8(0xB0, 0);
                                    //     txDataBuff.writeUInt8(0x00, 1);
                                    //     txCharacteristic.write(txDataBuff, true);
                                    // }, 10000);
                                }
                            }
                        });
                    }
                }
            });
        }
    });
});
