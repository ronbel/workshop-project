import {AsyncStorage} from 'react-native';

export default class StorageService{

    static _storeData = async (key,data) => {
        try {
          await AsyncStorage.setItem(key,JSON.stringify(data));
        } catch (err) {
            console.error('Error in store data', err.message);
        }
      };

    static _retrieveData = async (key) => {
        try {
          const value = JSON.parse(await AsyncStorage.getItem(key));
          if (value !== null) {
            // We have data!!
            console.warn(value);
            return value;
          }
        } catch (err) {
            console.error('Error in retrieve data', err.message);
        }
      };

    static set_contact=  async(contacts)=>{
      await  this._storeData("CONTACTS", contacts);
    }

    static get_contacts = async()=>{
      return  await this._retrieveData("CONTACTS");
    }

    static set_settings = async(settings)=>{
      await this._storeData("SETTINGS", settings);
    }

    static get_settings = async()=>{
      return await this._retrieveData("SETTINGS");
    }


}