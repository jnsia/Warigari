<template>
  <div class="main-back-ground">
    <header
      style="
        display: flex;
        flex-direction: row;
        justify-content: space-between;
        min-width: 1100px;
        margin-bottom: 10px;
      "
    >
      <div class="col-center">
        <font-awesome-icon
          :icon="['fas', 'chevron-left']"
          style="font-size: x-large; margin: 0px 5px"
          class="pointer"
          @click="goBack"
        />
      </div>
      <h1>현재 주행 상황</h1>
      <div></div>
    </header>

    <!-- 카메라 화면 -->
    <div class="col-center">
      <img
        id="ros-image"
        alt="ROS compressed image stream"
        style="width: 100%"
      />
      <div
        style="
          display: flex;
          flex-direction: row;
          justify-content: space-between;
        "
      >
        <button class="btn pointer" @click="goChangeLine">차선 변경</button>
        <button class="btn pointer" @click="goControl">속도 제어</button>
      </div>
      <div>
        <div v-if="front">전방에 장애물 또는 다른 차량이 있습니다. 속도를 감속합니다.</div>
        <div v-if="back">후방에 장애물 또는 다른 차량이 있습니다. 속도를 가속합니다.</div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref } from "vue";
import { useCounterStore } from "@/stores/counter";
import { useRouter } from "vue-router";
import { RouterLink, RouterView } from "vue-router";

const store = useCounterStore();

const front = ref(false);
const back = ref(false);

var imageListener = new ROSLIB.Topic({
  ros: store.ros,
  name: "/current_img",
  messageType: "sensor_msgs/msg/CompressedImage",
});

var lidarListener = new ROSLIB.Topic({
  ros: store.ros,
  name: "/object_detection",
  messageType: "my_py_interface/msg/Lidar",
});

imageListener.subscribe(function (message) {
  const src = "data:image/jpeg;base64," + message.data;
  document.getElementById("ros-image").src = src;
});

// lidarListener.subscribe(function (message) {
//   console.log(message)
//   front.value = message.front;
//   back.value = message.back;
// });

const router = useRouter();
const goChangeLine = () => {
  router.push({ name: "changeLine" });
};
const goControl = () => {
  router.push({ name: "control" });
};
const goBack = () => {
  router.go(-1);
};
</script>

<style scope></style>
