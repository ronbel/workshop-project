import {NativeModules, PermissionsAndroid} from 'react-native';
import storage from './storage-service';
import {getLocationUrl} from './location-service';

const DirectSms = NativeModules.DirectSms;

export default class SMSService {

    static sendSMS = async (number, message) => {
        try {
            await PermissionsAndroid.request(PermissionsAndroid.PERMISSIONS.SEND_SMS);
            DirectSms.sendDirectSms(number, message);
        } catch (err) {
            console.error('error in getting location', err.message);
        }
    };


    static getAndSend = async () => {
        try {
            let contactsArray = await storage.get_contacts();
            let settings = await storage.get_settings();
            let {message} = settings;
            if(settings.addLocation){
                message+=`\n\n${await getLocationUrl()}`
            }
            for (let contact of contactsArray) {
                await this.sendSMS(contact.number, message);
            }
        } catch (err) {
            console.error('error in getting contacts ', err.message);
        }

    };
}
