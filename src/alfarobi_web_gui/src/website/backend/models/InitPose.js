import mongoose from "mongoose";

const InitPose = mongoose.Schema({
  robot: {
    type: String,
    required: true,
  },
  data: {
    r_sho_r: {
      type: Number,
      required: true,
    },
    r_sho_p: {
      type: Number,
      required: true,
    },
    r_el: {
      type: Number,
      required: true,
    },
    r_hip_r: {
      type: Number,
      required: true,
    },
    r_hip_p: {
      type: Number,
      required: true,
    },
    r_hip_y: {
      type: Number,
      required: true,
    },
    r_knee: {
      type: Number,
      required: true,
    },
    r_ank_r: {
      type: Number,
      required: true,
    },
    r_ank_p: {
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
    l_sho_r: {
      type: Number,
      required: true,
    },
    l_sho_p: {
      type: Number,
      required: true,
    },
    l_el: {
      type: Number,
      required: true,
    },
    l_hip_r: {
      type: Number,
      required: true,
    },
    l_hip_p: {
      type: Number,
      required: true,
    },
    l_hip_y: {
      type: Number,
      required: true,
    },
    l_knee: {
      type: Number,
      required: true,
    },
    l_ank_r: {
      type: Number,
      required: true,
    },
    l_ank_p: {
      type: Number,
      required: true,
    },
  },
});

export default mongoose.model("InitPoses", InitPose);
