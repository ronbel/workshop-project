import {NativeModules , PermissionsAndroid} from 'react-native';
import storage from './storage-service';
import location from './location-service';
const DirectSms = NativeModules.DirectSms;

export default class SMSService{
    
     static sendSMS = async(element) =>{
        try{
            console.warn('here 2');
            let loc = await location;
            DirectSms.sendDirectSms(element, loc);
            console.warn('now sendind sms');
        }

        catch(err){
            console.error('error in getting location' , err.message);
        }}


     static getAndSend = async() => {
        //try{
        console.warn('here');
        //let contactsArray = await(storage.get_contacts());
        let contactsArray = ['+972502733733','+972525365046'];
        contactsArray.forEach(this.sendSMS);
        console.warn('now in send sms');
       // }

      //  catch(err){
       //     console.error('error in getting contacts ', err.message);
      //  }

    };
}
