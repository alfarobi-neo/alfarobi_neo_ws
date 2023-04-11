import express from "express";
import mongoose from "mongoose";
import cors from "cors";
import RobotRoute from "./routes/RobotRoute.js";

const app = express();
mongoose.connect("mongodb://admin:password@mongo", {
  useNewUrlParser: true,
  useUnifiedTopology: true,
});
const db = mongoose.connection;
db.on("error", (error) => console.log(error));
db.once("open", () => console.log("Database connected..."));
app.use(cors());
app.use(express.json());
app.use(RobotRoute);
app.listen(5000, () => console.log("Server up and running..."));
