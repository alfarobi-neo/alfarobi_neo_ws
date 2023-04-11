import React from "react";
import { useNavigate } from "react-router-dom";
import { useParams } from "react-router-dom";

function Sidebar() {
  const { robot } = useParams();
  // console.log(robot);
  let navigate = useNavigate();
  const Menus = [
    { title: "Motion", header: true },
    { title: "Initial Pose", nav: "/init/" + robot },
    { title: "Walking" ,},
    { title: "Kicking" },
    { title: "Sequential Movement",  nav: "/sequencer/" + robot },
    { title: "Vision", header: true },
    { title: "Nunggu vision" },
    { title: "Nunggu vision" },
    { title: "Nunggu vision" },
    { title: "Nunggu vision" },
    { title: "Others", header: true },
    { title: "Feedback Gains" },
    { title: "Teleoperation" },
    { title: "Sounds" },
  ];

  return (
    <div>
      <button
        className="w-[28px] h-fit text-white ml-5 absolute top-0 peer text-4xl"
        id="button_aside"
      >
        ≡
      </button>
      <aside
        className="grid fixed top-0 bg-[#232323] w-[233px] h-full -left-[233px] peer-focus:left-0 ease-out delay-150 duration-300 rounded-r-[10px]"
        id="aside"
      >
        <div>
          <div className="pt-6">
            <ul>
              {Menus.map((menu, index) => (
                <li
                  key={index}
                  onClick={() => {
                    navigate(`${menu.nav}`);
                  }}
                  className={`flex rounded-md px-6 text-white items-center gap-x-4 } ${
                    menu.header
                      ? "pt-8 -ml-2 font-medium text-xl duration-200"
                      : "cursor-pointer hover:bg-black m-2"
                  }`}
                >
                  <span className={`origin-left duration-200`}>
                    {menu.title}
                  </span>
                </li>
              ))}
            </ul>
          </div>
        </div>
      </aside>
    </div>
  );
}

export default Sidebar;
