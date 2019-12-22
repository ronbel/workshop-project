
import React, {useState} from 'react';
import {
    SafeAreaView,
    Button,
    StatusBar,
    Text,
    Alert,
    PermissionsAndroid,
    ToastAndroid
} from 'react-native';
import {BleManager} from 'react-native-ble-plx';
import {NativeModules} from 'react-native';
const DirectSms = NativeModules.DirectSms;

const BLEManager = new BleManager();

const charUUID = '0000fff4-0000-1000-8000-00805f9b34fb';



const App: () => React$Node = () => {
    const [foundMyDevice, setFoundMyDevice] = useState(false);
    const [char, setChar] = useState('');

    const addDevice = (err, device) => {
        if (err) {
            Alert.alert('Error', err.message);
        }
        if (device && device.name) {
            setFoundMyDevice(true);

            ToastAndroid.showWithGravity('Found LRRM', ToastAndroid.SHORT, ToastAndroid.CENTER)

            if (device.name.toLowerCase() === 'lrrm') {
                device.connect().then(async device => {
                    let fullDevice = await device.discoverAllServicesAndCharacteristics();
                    let services = await fullDevice.services();
                    for (let service of services) {
                        let chars = await service.characteristics();

                        if (chars.some(x => x.uuid === charUUID)) {
                            console.warn('Found char!');
                            let myChar = chars.find(x => x.uuid === charUUID);
                            setChar(myChar.value)
                            myChar.monitor((err, char) => {
                                console.warn('Sending SMS')
                                try {
                                    DirectSms.sendDirectSms('+972502733733', 'שלחתי SMS מצוקה!');
                                } catch (e) {
                                    console.error(e)
                                }
                            })
                        }


                    }
                }).catch(err => Alert.alert('Error', err.message));
            }
            BLEManager.stopDeviceScan();
        }
    };

    const startScan = async () => {
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_COARSE_LOCATION,
        );
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
        );
        await PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.SEND_SMS
        );

        BLEManager.startDeviceScan(null, null, addDevice);
    };
    return (
        <>
            <StatusBar barStyle="dark-content"/>
            <SafeAreaView
                style={{flex: 1, alignItems: 'center', justifyContent: 'center'}}>
                <Button title={'Scan'} onPress={startScan}/>
                {foundMyDevice && <Text>Yay, I found LRRM</Text>}
                <Text>Tracked char value is {char}</Text>
            </SafeAreaView>
        </>
    );
};

export default App;
