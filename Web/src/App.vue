<template>
  <div>
    <RouterView></RouterView>
  </div>
</template>

<script setup>
import { onMounted } from 'vue';
import { RouterLink, RouterView } from 'vue-router'
import { useCounterStore } from '../src/stores/counter';

const store = useCounterStore();

onMounted(()=>{
  try{
    store.ros = new ROSLIB.Ros({
      url: 'ws://192.168.100.215:9090'
    });
    store.ros.on('connection', function() {
      console.log('Connected to websocket server.');
    });

    store.ros.on('error', function(error) {
      console.log('Error connecting to websocket server: ', error);
    });
    store.ros.on('close', function() {
      console.log('Connection to websocket server closed.');
    });

  } catch(error){
      console.log(error)
  }
})
</script>

<style scoped>
</style>