import {PermissionsAndroid} from 'react-native';
import Contacts from 'react-native-contacts';


export const getContacts = () => {
    return new Promise((resolve, reject) => {
        PermissionsAndroid.request(
            PermissionsAndroid.PERMISSIONS.READ_CONTACTS).then( () => {
            Contacts.getAll((err, contacts) => {
                if(err){
                    reject(err);
                    return;
                }
                //Sort contacts alphabetically and get only name and phone, filter out nulls
                contacts = contacts.sort((c1,c2) => c1.displayName.localeCompare(c2.displayName)).map( contact => {
                    let numberObj = (contact.phoneNumbers.find(numberObj => numberObj.label==='mobile'));
                    return numberObj ? {name: contact.displayName, number: numberObj.number.replace(/[- ]/g, '')} : null;
                }).filter(x => x!==null);
                resolve(contacts);
            });
        });
    });
};

