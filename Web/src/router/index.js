import { createRouter, createWebHistory } from 'vue-router'
import MainView from '../views/MainView.vue'
import ChangeLineView from '../views/ChangeLineView.vue'
import NowDriveView from '../views/NowDriveView.vue'
import ControlView from '../views/ControlView.vue'


const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: 'main',
      component: MainView
    },
    {
      path: '/drive',
      name: 'drive',
      component: NowDriveView
    },
    {
      path: '/changeLine',
      name: 'changeLine',
      component: ChangeLineView
    },
    {
      path: '/control',
      name: 'control',
      component: ControlView
    },
  ]
})

export default router
