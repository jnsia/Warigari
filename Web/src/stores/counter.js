import { ref, computed } from 'vue'
import { defineStore } from 'pinia'

export const useCounterStore = defineStore('counter', () => {
  const ros = ref(null);
  const nowSpeed = ref(7);

  return { 
    ros,
    nowSpeed 
  }
},{ persist:true });
