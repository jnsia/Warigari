<template>
    <div class="main-back-ground" style="margin-bottom: 0px;">
        <h1 style="margin: 0px; font-size: 52px;">WARIGARI AUTOMOBILE SERVICE</h1>
        <p class="sub-title">Drive for Free</p>
        <img src="../assets/원격제어.png" alt="원격 제어" @click="goDrivingInfo" class="pointer" style="margin-top: 50px;">
        <footer id="footer">COPYRIGHT ⓒ WALIGALI MOTOR COMPANY. ALL RIGHTS RESERVED.</footer>
    </div>
</template>

<script setup>
import { RouterLink, RouterView, useRouter } from 'vue-router'

const router = useRouter()
const goDrivingInfo = () => {
    router.push({name:"drive"})
}
</script>

<style scoped>
.main-back-ground {
    padding: 20px;
    display: flex;
    justify-content: center;
}
.sub-title {
    font-size: 24px;
    color: #eeeeee;
    margin-top: 20px;
}
#footer {
    position: absolute;
    bottom: 0;
    padding-bottom: 20px;
}
</style>