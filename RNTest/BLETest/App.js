
import React, {useState} from 'react';
import {
    View,
    SafeAreaView,
    Button,
    StatusBar,
    Text,
    Alert,
    PermissionsAndroid,
    ToastAndroid,
    AsyncStorage,
    TextInput
} from 'react-native';
import {BleManager} from 'react-native-ble-plx';
import {NativeModules} from 'react-native';
import { Component } from 'react';
import Storage from './services/storage-service'
const DirectSms = NativeModules.DirectSms;

const BLEManager = new BleManager();

const charUUID = '0000fff4-0000-1000-8000-00805f9b34fb';


const App: () => React$Node = () => {
    const [foundMyDevice, setFoundMyDevice] = useState(false);
    const [char, setChar] = useState('');
    const [value, setValue] = useState('');
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

    const addContact = async (contact) => {
        console.warn('מוסיף איש קשר');
        

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
                <TextInput
                    style={{width:300, height:50, borderWidth:1}}
                    onChangeText={setValue}
                    value={value}       
                />
                <View style={{height:100, justifyContent: 'space-between'}}>
                <Button title={'Add contact'} onPress={() =>Storage.set_contact(value)}/>
                <Button title={'Get contacts'} onPress={() => Storage.get_contacts().then(res => console.warn(res))}/>
                </View>
                <Text>Tracked char value is {char}</Text>
                <Text>Number to send to: +972502733733!</Text>
            </SafeAreaView>
        </>
    );
};

export default App;
