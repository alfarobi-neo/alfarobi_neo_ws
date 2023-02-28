import { useNavigate } from "react-router-dom";
import logo from "../assets/alfarobi_logo_head.png";

function RobotPickerButton(props) {
  let navigate = useNavigate();
  return (
    // todo href and connected || disconnected state
    <div
      onClick={() => {
        navigate("/tune/" + props.children);
      }}
      className="flex flex-col items-center bg-[#E9E9E9] hover:bg-[#464646] mx-[1.5vw] py-[2.5vh] px-[3vw] rounded-lg hover:cursor-pointer"
    >
      <img className="h-[12vw]" src={logo} alt="logo" />
      <p className="font-bold text-[5vw] my-[3vh]">{props.children}</p>
      <p className="font-bold text-[#008B16]">connected</p>
    </div>
  );
}

export default RobotPickerButton;
