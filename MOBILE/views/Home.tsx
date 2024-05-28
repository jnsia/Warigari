import React from 'react';
import {
  Image,
  SafeAreaView,
  StatusBar,
  StyleSheet,
  Text,
  TouchableOpacity,
  useColorScheme,
  View,
} from 'react-native';

import {
  ScrollView
} from 'react-native-gesture-handler'

import {Colors} from 'react-native/Libraries/NewAppScreen';
import ROSLIB from 'roslib';

const Home = () => {
  const isDarkMode = useColorScheme() === 'dark';

  const backgroundStyle = {
    backgroundColor: isDarkMode ? Colors.darker : Colors.lighter,
  };

  const ros = new ROSLIB.Ros({});

  ros.connect('ws://192.168.100.215:9090');

  ros.on('error', error => {
    console.log(error);
  });

  ros.on('connection', error => {
    console.log('Connection: ok!');
  });

  ros.on('close', error => {
    console.log('Connection closed.');
  });

  const topic = new ROSLIB.Topic({
    ros: ros,

    name: '/topic',

    messageType: 'std_msgs/String',
  });

  let msg = new ROSLIB.Message({
    data: '',
  });

  // function publish() {
  //   msg.data = document.getElementById("msg").value;

  //   document.getElementById("msg").value = "";

  //   topic.publish(msg);
  // }

  // window.onload = () => {
  //   topic.subscribe((res) => {
  //     console.log(res)
  //     document.getElementById("subscribe").innerHTML = res.data;
  //   });
  // };

  return (
    <SafeAreaView style={backgroundStyle}>
      <StatusBar
        barStyle={isDarkMode ? 'light-content' : 'dark-content'}
        backgroundColor={backgroundStyle.backgroundColor}
      />
      <ScrollView
        // contentInsetAdjustmentBehavior="automatic"
        style={{
          height: '100%',
          backgroundColor: '#0B3247',
        }}>
        <View style={{
          marginTop: 20,
          justifyContent: 'center',
          alignItems: 'center'
        }}>
          <TouchableOpacity style={styles.sectionContainer}>
            <Image
              style={styles.sectionImage}
              source={require('../assets/images/CrossLineIcon.png')}></Image>
            <Text style={styles.highlight}>차선변경</Text>
          </TouchableOpacity>
          <TouchableOpacity style={styles.sectionContainer}>
            <Image
              style={styles.sectionImage}
              source={require('../assets/images/FreeParkingIcon.png')}></Image>
            <Text style={styles.highlight}>자동주차</Text>
          </TouchableOpacity>
          <TouchableOpacity style={styles.sectionContainer}>
            <Image
              style={styles.sectionImage}
              source={require('../assets/images/AutoControlIcon.png')}></Image>
            <Text style={styles.highlight}>차량 원격 제어</Text>
          </TouchableOpacity>
        </View>
      </ScrollView>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  sectionContainer: {
    marginTop: 40,
    padding: 24,
    width: 200,
    height: 160,
    backgroundColor: '#ffffff',
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    gap: 10,
    borderRadius: 10,
  },
  sectionImage: {
    width: 48,
    height: 48,
    objectFit: 'contain',
  },
  sectionDescription: {
    marginTop: 8,
    fontSize: 18,
    fontWeight: '400',
  },
  highlight: {
    color: '#000000',
    fontWeight: '700',
    fontSize: 24,
  },
});

export default Home;
