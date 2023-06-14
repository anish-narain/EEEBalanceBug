//React props are used to pass parameters
//to reach components
//Read more about React Props here
//https://www.w3schools.com/react/react_props.asp;

function TableComp(props) {
  ///Map each col to its corresponding HTML
  function TableCols(props) {
    const cols = props.tr.map((col) => <td>{col}</td>);
    return cols;
  }
  //Map each row to its corresponding HTML
  function TableRows(props) {
    const rows = props.td.map((row) => (
      <tr>
        <TableCols tr={row} />
      </tr>
    ));
    return rows;
  }
  return (
    <div className="TableComp">
      <table>
        <TableRows td={props.td} />
      </table>
    </div>
  );
}

export default TableComp;
