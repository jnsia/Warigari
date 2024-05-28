<template>
    <div class="main-back-ground">
        <header style="display: flex; flex-direction: row; justify-content: space-between; min-width: 1100px;">
            <div style="display: flex; flex-direction: column; justify-content: center;"><font-awesome-icon :icon="['fas', 'chevron-left']" style="font-size: x-large; margin: 0px 5px;" class="pointer" @click="goBack"/></div>
            <h1>차량 속도 제어</h1>
            <div></div>
        </header>
        <div class="col-center">
            <img
                id="ros-image"
                alt="ROS compressed image stream"
                style="width: 100%"
            />
            <div style="display: flex; flex-direction:row; justify-content: space-between; align-items: center; width: 200px;">
                <!-- 연산 필요, 속도 10 증가하려면, 전압 얼마 줘야하는지 알아야함 -->
                <button class="btn pointer" @click="changeSpeed(1)">+</button>
                <p>{{store.nowSpeed}}</p>
                <button class="btn pointer" @click="changeSpeed(-1)">-</button>
            </div>
        </div>
    </div>
</template>

<script setup>
import { useCounterStore } from '@/stores/counter';
import { useRouter } from 'vue-router';


const router = useRouter();
const store = useCounterStore();

// 카메라 이미지 topic
var listener = new ROSLIB.Topic({
  ros: store.ros,
  name: "/current_img",
  messageType: "sensor_msgs/msg/CompressedImage",
});
// 카메라 이미지 subscribe
listener.subscribe(function (message) {
  var src = "data:image/jpeg;base64," + message.data;
  document.getElementById("ros-image").src = src;
  console.log(document.getElementById("ros-image").src)
});

// 속도 topic
var speed_talker = new ROSLIB.Topic({
  ros: store.ros,
  name: "/speed_control",
  messageType: "std_msgs/Int8",
});

// 속도 메세지
var message = new ROSLIB.Message({
  data: store.nowSpeed,
});

// 차량 속도 제어
const changeSpeed = (num) => {
    store.nowSpeed += num;

    message.data = num;
    console.log(message);
    speed_talker.publish(message);
};

const goBack = () => {
    router.go(-1);
}
</script>

<style scoped>
</style>