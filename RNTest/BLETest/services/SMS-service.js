import {NativeModules, PermissionsAndroid} from 'react-native';
import storage from './storage-service';
import {getLocationUrl} from './location-service';

const DirectSms = NativeModules.DirectSms;

export default class SMSService {

    static sendSMS = async (number, location) => {
        try {
            await PermissionsAndroid.request(PermissionsAndroid.PERMISSIONS.SEND_SMS);
            DirectSms.sendDirectSms(number, location);
        } catch (err) {
            console.error('error in getting location', err.message);
        }
    };


    static getAndSend = async () => {
        try {
            let contactsArray = await (storage.get_contacts());
            let loc = await getLocationUrl();
            for (let contact of contactsArray) {
                await this.sendSMS(contact.number, loc);
            }

        } catch (err) {
            console.error('error in getting contacts ', err.message);
        }

    };
}
