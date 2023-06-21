function primeOrNot() {
  let num = Math.floor(document.getElementById("num1").value);
  let response = "The number is a prime";
  if (num < 2) {
    response = "The number is not a prime";
  } else if (num > 2) {
    const rootnum = Math.sqrt(num);
    for (let d = 2; d < rootnum; d++) {
      if (num % d == 0) {
        response = "The number is not a prime";
      }
    }
  }
  document.getElementById("response").innerHTML = response;
}
