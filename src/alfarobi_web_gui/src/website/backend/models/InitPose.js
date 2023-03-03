import mongoose from "mongoose";

const InitPose = mongoose.Schema({
  robot: {
    type: String,
    required: true,
  },
  data: {
    r_sho_roll: {
      type: Number,
      required: true,
    },
    r_sho_pitch: {
      type: Number,
      required: true,
    },
    r_el_pitch: {
      type: Number,
      required: true,
    },
    r_hip_roll: {
      type: Number,
      required: true,
    },
    r_hip_pitch: {
      type: Number,
      required: true,
    },
    r_hip_yaw: {
      type: Number,
      required: true,
    },
    r_knee_pitch: {
      type: Number,
      required: true,
    },
    r_ank_roll: {
      type: Number,
      required: true,
    },
    r_ank_pitch: {
      type: Number,
      required: true,
    },
    head_tilt: {
      type: Number,
      required: true,
    },
    head_pan: {
      type: Number,
      required: true,
    },
    l_sho_roll: {
      type: Number,
      required: true,
    },
    l_sho_pitch: {
      type: Number,
      required: true,
    },
    l_el_pitch: {
      type: Number,
      required: true,
    },
    l_hip_roll: {
      type: Number,
      required: true,
    },
    l_hip_pitch: {
      type: Number,
      required: true,
    },
    l_hip_yaw: {
      type: Number,
      required: true,
    },
    l_knee_pitch: {
      type: Number,
      required: true,
    },
    l_ank_roll: {
      type: Number,
      required: true,
    },
    l_ank_pitch: {
      type: Number,
      required: true,
    },
  },
});

export default mongoose.model("InitPoses", InitPose);
