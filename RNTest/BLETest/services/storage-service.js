import {AsyncStorage} from 'react-native';

export default class StorageService{

    static _storeData = async (data) => {
        try {
          await AsyncStorage.setItem("1",data);
        } catch (err) {
            console.error('Error in store data', err.message);
        }
      };

    static _retrieveData = async () => {
        try {
          const value = await AsyncStorage.getItem('CONTACTS');
          if (value !== null) {
            // We have data!!
            console.warn(value);
          }
        } catch (err) {
            console.error('Error in retrieve data', err.message);
        }
      };



}