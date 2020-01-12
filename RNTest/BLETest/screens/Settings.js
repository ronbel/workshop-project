import React from 'react';
import {View, Text, TextInput, StyleSheet, Switch} from 'react-native';
import SettingsSection from '../components/SettingsSection';
import NumericInput from 'react-native-numeric-input';

export default function Settings() {


    return (
        <View style={{flex: 1, paddingVertical: 20, alignItems: 'center'}}>
            <Text style={{fontSize: 25, marginBottom: 20}}>Settings</Text>

            <SettingsSection title="Message">
                <TextInput multiline style={styles.messageInput}
                           placeholder="Write the message that will be sent to your contacts here"/>
            </SettingsSection>

            <SettingsSection title="Location">
                <Text>Add my location to the message</Text>
                <Switch/>
            </SettingsSection>

            <SettingsSection title="False Alarms">

                <Text style={{width: 200}}>{'Seconds to wait before sending\n\n'}
                    <Text style={{fontSize: 10, color: 'grey'}}>It's recommended to give yourself enough time to respond
                        to false alarms</Text>
                </Text>

                <NumericInput valueType="integer" initValue={5} minValue={2} maxValue={60} type="plus-minus" onChange={() => {
                }}/>
            </SettingsSection>
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
