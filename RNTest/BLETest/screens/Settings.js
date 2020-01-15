import React, {useState, useEffect, useRef} from 'react';
import {View, Text, TextInput, StyleSheet, Switch, ActivityIndicator, Button, Alert, ToastAndroid} from 'react-native';
import SettingsSection from '../components/SettingsSection';
import NumericInput from 'react-native-numeric-input';
import Storage from '../services/storage-service';
import ModalDropdown from 'react-native-modal-dropdown';
import BLE from '../services/ble-service';
import LoadingModal from '../components/LoadingModal';


export default function Settings() {
    const [isReady, setIsReady] = useState(false);
    const [message, setMessage] = useState('');
    const [addLocation, setAddLocation] = useState(false);
    const [secondsToWait, setSecondsToWait] = useState(5);
    const [idleMinutes, setIdleMinutes] = useState(30);
    const [isSaving, setIsSaving] = useState(false);


    useEffect(() => {
        Storage.get_settings().then(res => {
            setMessage(res.message);
            setAddLocation(res.addLocation);
            setSecondsToWait(res.secondsToWait);
            setIdleMinutes(res.idleMinutes);
            setIsReady(true);
        })
    }, []);

    const saveSettings = async () => {
        if(message.trim() === ''){
            Alert.alert('Error', 'Your message cannot be empty!');
            return;
        }
        try {
            setIsSaving(true);
            await BLE.writeToIdleMinutesChar(Number.parseInt(idleMinutes));
            await Storage.set_settings({message, addLocation, secondsToWait, idleMinutes});
            ToastAndroid.showWithGravity('Settings saved successfully', ToastAndroid.LONG, ToastAndroid.CENTER)
        }catch (e) {
            Alert.alert('Error', 'There was a problem saving the settings, please try again!' + `\n${e.message}`)
        } finally {
            setIsSaving(false);
        }
    };


    if (!isReady) {
        return (
            <View style={{flex: 1, justifyContent: 'center', alignItems: 'center'}}>
                <ActivityIndicator size="large"/>
            </View>
        );
    }

    return (
        <View style={{flex: 1, paddingVertical: 20, alignItems: 'center'}}>
            <LoadingModal visible={isSaving}/>
            <Text style={{fontSize: 25, marginBottom: 20}}>Settings</Text>

            <SettingsSection title="Message">
                <TextInput value={message} onChangeText={setMessage} multiline style={styles.messageInput}
                           placeholder="Write the message that will be sent to your contacts here"/>
            </SettingsSection>

            <SettingsSection title="Location">
                <Text>Add my location to the message</Text>
                <Switch value={addLocation} onValueChange={setAddLocation}/>
            </SettingsSection>

            <SettingsSection title="False Alarms">

                <Text style={{width: 200}}>{'Seconds to wait before sending\n\n'}
                    <Text style={{fontSize: 10, color: 'grey'}}>It's recommended to give yourself enough time to respond
                        to false alarms</Text>
                </Text>

                <NumericInput valueType="integer" value={secondsToWait} initValue={secondsToWait} onChange={setSecondsToWait} minValue={2} maxValue={60} type="plus-minus"/>
            </SettingsSection>

            <SettingsSection title="Idle Time">
                <Text style={{width: 200}}>Minutes with no movement to wait until message is sent</Text>
                <ModalDropdown showsVerticalScrollIndicator={false} onSelect={(index, value) => setIdleMinutes(value)} dropdownStyle={{transform:[{translateX: 40}]}} defaultValue={idleMinutes} textStyle={{width: '100%', height: '100%', fontSize: 20, textAlignVertical: 'center'}} style={{borderWidth: 1, height: 40, width: 100, justifyContent: 'center', alignItems: 'center'}} options={[15,30,60,90,120]}/>
            </SettingsSection>


            <Button title="Save Settings" onPress={saveSettings}/>
        </View>
    );

}

const styles = StyleSheet.create({
    messageInput: {
        width: '95%',
        height: 90,
        padding: 10,
        textAlignVertical: 'top',
        borderWidth: 0.3,
        borderRadius: 15,
    },
    secondsInput: {
        borderWidth: 0.3,
        borderRadius: 15,
        textAlign: 'center',
    },
});
