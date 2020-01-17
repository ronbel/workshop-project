import React, {useState, useEffect} from 'react';
import {TouchableOpacity, TextInput, ToastAndroid, Alert, View, Text, ActivityIndicator, ScrollView, StyleSheet, CheckBox, Button} from 'react-native';
import {getContacts} from '../services/contacts-service';
import Storage from '../services/storage-service';
import LoadingModal from '../components/LoadingModal';


export default function Contacts() {
    const [contactList, setContactList] = useState([]);
    const [selectedContacts, setSelectedContacts] = useState([]);
    const [isReady, setIsReady] = useState(false);
    const [searchWord, setSearchWord] = useState('');
    const [isSaving, setIsSaving] = useState(false);
    const [showOnlySelected, setShowOnlySelected] = useState(false);

    useEffect(() => {
        Promise.all([getContacts(), Storage.get_contacts()]).then(res => {
            setContactList(res[0]);
            setSelectedContacts(res[1]||[]);
            setIsReady(true);
        })
    }, []);

    const isContactSelected = (contact) => selectedContacts.findIndex(x => x.name === contact.name) !== -1;
    const removeContact = (contact) => setSelectedContacts(selectedContacts.filter(x => x.name !== contact.name));
    const addContact = contact => setSelectedContacts([...selectedContacts, contact]);

    const onContactPressed = (contact) => {
        isContactSelected(contact) ? removeContact(contact) : addContact(contact);
    };

    const saveContacts = async () => {
        try{
            setIsSaving(true);
            await Storage.set_contact(selectedContacts);
            ToastAndroid.showWithGravity('Contacts saved successfully', ToastAndroid.LONG, ToastAndroid.CENTER);
        } catch (e) {
            Alert.alert('Error', 'There was an error while saving the contacts, please try again')
        } finally {
            setIsSaving(false);
        }
    };

    const toggleOnlySelected = () => setShowOnlySelected(!showOnlySelected);

    if (!isReady) {
        return (
            <View style={{flex: 1, justifyContent: 'center', alignItems: 'center'}}>
                <ActivityIndicator size="large"/>
            </View>
        );
    }

    return (
        <View style={{flex: 1}}>
            <Text style={{fontSize: 25, marginVertical: 20, alignSelf: 'center'}}>Contacts</Text>
            <LoadingModal visible={isSaving}/>
            <View style={styles.scrollHeader}>
                <TextInput onChangeText={setSearchWord} placeholder="Search" style={styles.searchInput}/>
                <TouchableOpacity onPress={toggleOnlySelected}>
                    <View style={{width: 70, borderWidth: 0.5, borderRadius: 5, backgroundColor: showOnlySelected ? 'green' : 'white'}}>
                        <Text style={{textAlign: 'center'}}>Show Selected</Text>
                    </View>
                </TouchableOpacity>
            </View>
            <ScrollView style={{flex: 1}}>
                {
                    (showOnlySelected ? selectedContacts : contactList).filter(contact => contact.name.toLowerCase().includes(searchWord.toLowerCase())).map((contact, index) => (
                        <TouchableOpacity activeOpacity={0.8} key={contact.name} onPress={() => onContactPressed(contact)}>
                            <View style={styles.contactItem}>
                                <CheckBox onValueChange={() => onContactPressed(contact)}
                                          value={isContactSelected(contact)}/>
                                <Text>{contact.name}</Text>
                            </View>
                        </TouchableOpacity>
                    ))
                }
            </ScrollView>
            <Button title="Save Contacts" onPress={saveContacts}/>
        </View>
    );
}

const styles = StyleSheet.create({
    contactItem: {
        height: 50,
        alignItems: 'center',
        paddingHorizontal: 20,
        borderBottomWidth: 0.5,
        flexDirection: 'row',
    },
    scrollHeader: {
        height: 70,
        flexDirection: 'row',
        justifyContent: 'center',
        alignItems: 'center',
        padding: 15
    },
    searchInput: {
        width: 250,
        height: 40,
        padding: 10,
        borderWidth: 1,
        borderRadius: 10,
        marginRight: 15
    }
});
