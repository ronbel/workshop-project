import {BleManager} from 'react-native-ble-plx';
import {PermissionsAndroid} from 'react-native';
import Storage from './storage-service';
import {notify} from './notification-service';


const SERVICE_UUID = '0000fff0-0000-1000-8000-00805f9b34fb';
const CHAR_UUID = '0000fff4-0000-1000-8000-00805f9b34fb';
const IDLE_MINUTES_CHAR = '0000fff1-0000-1000-8000-00805f9b34fb';

const NUMBERS_IN_BASE_64 = {
    15: 'Dw==',
    30: 'Hg==',
    60: 'PA==',
    90: 'Wg==',
    120: 'eA==',
    180: 'wrQ=',
    240: 'w7A=',
};


export default class BLE {
    static BLEManager = new BleManager();
    static BLEDevice = null;
    static DeviceSub = null;

    static scanForDevice = async (callback) => {
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_COARSE_LOCATION,
        );
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
        );
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.SEND_SMS,
        );

        this.BLEManager.startDeviceScan(null, null, callback);
    };

    static stopScan = () => {
        this.BLEManager.stopDeviceScan();
    };

    static connectAndMonitorDevice = async (device, callback) => {
        let connectedDevice = await device.connect();
        let discoveredDevice = await connectedDevice.discoverAllServicesAndCharacteristics();
        BLE.BLEDevice = discoveredDevice;
        BLE.DeviceSub = discoveredDevice.monitorCharacteristicForService(SERVICE_UUID, CHAR_UUID, () => {
            notify();
            callback();
        });
        let settings = await Storage.get_settings();
        await discoveredDevice.writeCharacteristicWithResponseForService(SERVICE_UUID, IDLE_MINUTES_CHAR, NUMBERS_IN_BASE_64[settings.idleMinutes]);
    };

    static isPoweredOn = async () => {
        return ((await this.BLEManager.state()) === 'PoweredOn');
    };

    static writeToIdleMinutesChar = async (minutes) => {
        return BLE.BLEDevice.writeCharacteristicWithResponseForService(SERVICE_UUID, IDLE_MINUTES_CHAR, NUMBERS_IN_BASE_64[minutes]);

    };


}
