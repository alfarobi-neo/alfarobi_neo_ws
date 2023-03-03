import { useNavigate } from "react-router-dom";

function LandingPageButton(props) {
  let navigate = useNavigate();
  return (
    //todo add href
    <div
      onClick={() => {
        navigate(props.children === "per-robot tuning" ? "/pick" : "/");
      }}
      className="flex flex-row items-center bg-[#B0ECFF] hover:bg-black text-black hover:text-[#B0ECFF] w-1/4 mx-4 px-4 py-[4vh] rounded-lg hover:cursor-pointer"
    >
      <p className="italic text-xl md:text-3xl">{props.children}</p>
      <svg
        xmlns="http://www.w3.org/2000/svg"
        fill="none"
        viewBox="0 0 24 24"
        strokeWidth={1.5}
        stroke="currentColor"
        className="w-6 h-6"
      >
        <path
          strokeLinecap="round"
          strokeLinejoin="round"
          d="M8.25 4.5l7.5 7.5-7.5 7.5"
        />
      </svg>
    </div>
  );
}

export default LandingPageButton;
