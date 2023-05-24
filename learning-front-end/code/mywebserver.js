var http = require('http');
var server = http.createServer(function(req, res){
res.writeHead(200, {'Content-Type':'text/plain'})
res.end('You have reached your AWS EC2 web server...');
});
console.log('Server is running on port 3000');
server.listen(3000,'0.0.0.0');