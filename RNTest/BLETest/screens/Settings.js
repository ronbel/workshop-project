import React, {useState, useEffect} from 'react';
import {View, Text, TextInput, StyleSheet, Switch, ActivityIndicator, Button, Alert, ToastAndroid} from 'react-native';
import SettingsSection from '../components/SettingsSection';
import NumericInput from 'react-native-numeric-input';
import Storage from '../services/storage-service';


export default function Settings() {
    const [isReady, setIsReady] = useState(false);
    const [message, setMessage] = useState('');
    const [addLocation, setAddLocation] = useState(false);
    const [secondsToWait, setSecondsToWait] = useState(5);

    useEffect(() => {
        Storage.get_settings().then(res => {
            setMessage(res.message);
            setAddLocation(res.addLocation);
            setSecondsToWait(res.secondsToWait);
            setIsReady(true);
        })
    }, []);

    const saveSettings = async () => {
        if(message.trim() === ''){
            Alert.alert('Error', 'Your message cannot be empty!');
            return;
        }
        try {
            await Storage.set_settings({message, addLocation, secondsToWait});
            ToastAndroid.showWithGravity('Settings saved successfully', ToastAndroid.LONG, ToastAndroid.CENTER)
        }catch (e) {
            Alert.alert('Error', 'There was a problem saving the settings, please try again!')
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
