<template>
  <div class="main-back-ground">
    <header
      style="
        display: flex;
        flex-direction: row;
        width: 100%;
        max-width: 1100px;
        padding: 0px 20px;
        justify-content: space-between;
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
      <div style="z-index: 1; padding: 10px 25px">
        <h1 style="color: white">차선 변경</h1>
      </div>
      <div></div>
    </header>
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
        <button
          :class="lineChanging != -1 ? 'disable-btn' : 'btn'"
          :disabled="lineChanging != -1"
          @click="goChangeLine('left')"
        >
          왼쪽 차선
        </button>
        <button
          :class="lineChanging != -1 ? 'disable-btn' : 'btn'"
          :disabled="lineChanging != -1"
          @click="goChangeLine('right')"
        >
          오른쪽 차선
        </button>
      </div>
      <div>
        <div v-if="lineChanging == 1">차선 변경 중입니다!</div>
        <div v-else-if="lineChanging == -1">차선 변경 대기 중입니다.</div>
        <div v-else>차선 변경을 완료하였습니다.</div>
      </div>
      <div>
        <div v-if="left">좌측에 장애물 또는 다른 차량이 있습니다. 차선 변경 기능이 제한됩니다.</div>
        <div v-if="right">우측에 장애물 또는 다른 차량이 있습니다. 차선 변경 기능이 제한됩니다.</div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, onMounted } from "vue";
import { useCounterStore } from "@/stores/counter";
import { useRouter } from "vue-router";

const store = useCounterStore();

const lineChanging = ref(-1);

const left = ref(false);
const right = ref(false);

var imageListener = new ROSLIB.Topic({
  ros: store.ros,
  name: "/current_img",
  messageType: "sensor_msgs/msg/CompressedImage",
});

var lineChangeListener = new ROSLIB.Topic({
  ros: store.ros,
  name: "/cam_to_num",
  messageType: "std_msgs/msg/Int16MultiArray",
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

lineChangeListener.subscribe(function (message) {
  const [ x1, x2, signal ] = message.data;

  if (lineChanging.value == 1 && signal == 0) {
    lineChanging.value = 0;

    setTimeout(() => {
      lineChanging.value = -1;
    }, 3000);
  } else if (lineChanging.value == -1 && signal == 0) {
    lineChanging.value = -1
  } else {
    lineChanging.value = signal
  }
});

// lidarListener.subscribe(function (message) {
//   left.value = message.left;
//   right.value = message.right;
// });

var talker = new ROSLIB.Topic({
  ros: store.ros,
  name: "/line_change",
  messageType: "std_msgs/String",
});

var message = new ROSLIB.Message({
  data: "Hello ROS!",
});

const router = useRouter();
const goChangeLine = (msg) => {
  message.data = msg;
  console.log(message);
  // talker.publish(JSON.stringify(message));
  talker.publish(message);
};
const goBack = () => {
  router.go(-1);
};
</script>

<style scope></style>
