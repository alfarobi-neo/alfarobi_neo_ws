import express from "express";
import { getInit, saveInit } from "../controllers/RobotController.js";
const router = express.Router();

router.get("/init", getInit);
router.post("/init", saveInit);

export default router;
