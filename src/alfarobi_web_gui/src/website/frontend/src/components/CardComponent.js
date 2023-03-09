import { React, useState, useEffect } from "react";
import ROSLIB from "roslib";
import { connect } from "react-redux";
// import { robots } from "../redux/GlobalState";
import { useSelector } from "react-redux";
import robotStore from "../redux/GlobalState";

function CardComponent(props) {
  const robot = useSelector(() => robotStore);
  const robotState = robot.getState()[`${props.robot}`];
  const [last, setLast] = useState(0.0);
  const [edited, setEdited] = useState(0.0);
  const [saved, setSaved] = useState(0.0);

  useEffect(() => {
    console.log("card component");
    // console.log(robotState[`${props.children}`]);
    setSaved(robotState[`${props.children}`]);
    //lewat
  }, []);

  // var listener = new ROSLIB.Topic({
  //   ros: ros,
  //   name: "/roll",
  //   messageType: "alfarobi_web_gui/ActionModule",
  // });

  return (
    <div className="bg-secondary_bg mx-1 py-0.5  mt-1 rounded">
      <div className="flex flex-row items-center text-center">
        <p className="w-[7vw] text-[12px]">{props.children}</p>
        <input
          type="number"
          step="0.01"
          className="w-[6vw] h-[2vh] ml-[0.3vw]"
          value={last}
          onChange={(event) => {
            setLast(event.target.value);
          }}
        />
        <input
          type="number"
          step="0.01"
          className="w-[6vw] h-[2vh] ml-[1.8vw]"
          value={edited}
          onChange={(event) => {
            props.handleAdd(
              `${props.children}`,
              props.robot,
              event.target.value
            );
            setEdited(event.target.value);
          }}
        />
        <input
          type="number"
          step="0.01"
          className="w-[6vw] h-[2vh] ml-[1.1vw]"
          value={saved}
          onChange={(event) => {
            // props.handleAdd(`${props.children}`, event.target.value);
            setSaved(event.target.value);
            // console.log("value is:", event.target.value);
          }}
        />
      </div>
    </div>
  );
}

const mapStateToProps = (state) => {
  return {
    state: state,
  };
};

const mapDispatchToProps = (dispatch) => {
  return {
    handleAdd: (type, robot, value) => {
      console.log("type " + robot);
      dispatch({ type: type, robot: robot, value: value });
    },
  };
};

export default connect(mapStateToProps, mapDispatchToProps)(CardComponent);
