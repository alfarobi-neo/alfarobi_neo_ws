import { configureStore, applyMiddleware } from "@reduxjs/toolkit";
import thunk from "redux-thunk";
const robotState = {
  r_sho_r: 0.0,
  r_sho_p: 0.0,
  r_el: 0.0,
  r_hip_r: 0.0,
  r_hip_p: 0.0,
  r_hip_y: 0.0,
  r_knee: 0.0,
  r_ank_r: 0.0,
  r_ank_p: 0.0,
  head_tilt: 0.0,
  head_pan: 0.0,
  l_sho_r: 0.0,
  l_sho_p: 0.0,
  l_el: 0.0,
  l_hip_r: 0.0,
  l_hip_p: 0.0,
  l_hip_y: 0.0,
  l_knee: 0.0,
  l_ank_r: 0.0,
  l_ank_p: 0.0,
};

const robots = {
  Abi: robotState,
  Alfa: robotState,
  Robi: robotState,
};

const robotReducer = (state = robots, action) => {
  // console.log(action.robot + " " + action.type);
  if (action.type === "GET_INIT") {
    var value = action.value;
    return {
      ...state,
      [`${action.robot}`]: value,
    };
  }
  var value = parseFloat(action.value);
  return {
    ...state,
    [`${action.robot}`]: {
      ...state[`${action.robot}`],
      [`${action.type}`]: value,
    },
  };
};

const robotStore = configureStore({
  reducer: robotReducer,
  preloadedState: robots,
});

applyMiddleware(thunk)(robotStore);

export default robotStore;
