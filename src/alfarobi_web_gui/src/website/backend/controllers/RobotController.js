import InitPose from "../models/InitPose.js";

/**
 * Datanya sbb:
 * {robot:"Abi",
 *  data: {
 *   r_sho_roll:
 * }
 * }
 * @param {*} res
 */
export const getInit = async (req, res) => {
  try {
    const robot = await InitPose.findOne({ robot: req.query.robot });
    res.json(robot);
  } catch (error) {
    res.status(500).json({ message: error.message });
  }
};

export const saveInit = async (req, res) => {
  const newInit = { robot: req.query.robot, data: req.body };
  const init = new InitPose(newInit);
  try {
    const robot = await InitPose.findOne({ robot: req.query.robot });
    if (robot === null) {
      const insertedInit = await init.save();
    } else {
      const updateInit = await InitPose.updateOne(
        { robot: req.query.robot },
        { data: req.body }
      );
    }
    res.status(201).json({ message: "success" });
  } catch (error) {
    res.status(400).json({ message: error.message });
  }
};
