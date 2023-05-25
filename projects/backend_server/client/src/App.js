// client/src/App.js
import React, {useState} from "react";
import TableComp from './TableComp';
//App is a React Function Component
//Read about it, and React Components in general, here:
//https://www.w3schools.com/REACT/react_components.asp
function App() {
const [tableData33, updateTable33] = useState([[]]);
//Get initial input from the server
//Whenever App is invoked
//a 'side effect' is that
//the initial table data is fetched from the server
//useEffect is a hook used to associate such 'side effects'
//with components
React.useEffect(() => {
    ///See CORS
    fetch("http://localhost:3001/personQuery/")
    .then((res) => res.json())
    .then((data) => alert(JSON.stringify(data)))
    .catch((err) => alert(err)
    );
    }, []);
//handleClick is our event handler for the button click
const handleClick = (updateMethod) => {
fetch("http://localhost:3001/tableData33/")
.then((res) => res.json())
.then((data) => updateMethod(data.tableData33))
.catch((err) => alert(err)
);
};
return (
<div className="App">
<TableComp td = {tableData33}/>
<button onClick={() => handleClick(updateTable33)}>Randomize
ages</button>
</div>
);
}
export default App;