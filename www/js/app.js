/* ------------------------------
  - To avoid exposing the ros variable, we maintain a set of function.
  These methods are partially applied, that is, they
  are functions that use the ros variable but do not require it as an argument.
  ------------------------------ */
var call_service = undefined;

var get_services_1_list = undefined;
var get_services_2_list = undefined;
var services_1_listener = undefined;
var services_2_listener = undefined;

var get_launch_1_list = undefined;
var get_launch_2_list = undefined;
var launch_1_listener = undefined;
var launch_2_listener = undefined;

var get_wnodes_1_list = undefined;
var wnodes_1_listener = undefined;

/* ------------------------------
  - With result function we can easily call a service 
  - Result function parameters are :
      name : service name
      type : service type
      params : service parameters
      success_cb : function to call if service call was successfull
      error_cb : function to call if service call has not been successfull
  ------------------------------ */
function call_service_builder (ros)
{
  return function (name, type, params, success_cb, error_cb)
  {
    let srv = new ROSLIB.Service({
      ros: ros,
      name: name,
      serviceType: type
    });

    let request = new ROSLIB.ServiceRequest(params);

    srv.callService(request, success_cb, error_cb);
  }
}

/* ------------------------------------------------------------------------------------------
  - Connect utility
  ------------------------------------------------------------------------------------------ */

/* ------------------------------
  - Check if form input is valid. If it is, try to connect to ros
  ------------------------------ */
function validate_connection (form)
{
  let conn_data = {};

  for (let i = 0; i < form.elements.length; i++)
  {
    if (form.elements[i].name && form.elements[i].value)
      conn_data[form.elements[i].name] = form.elements[i].value;
  }
  // rosbridge port is 9090
  conn_data.port = '9090';

  if (conn_data.address && conn_data.port)
  {
    connect_to_ros(conn_data);
  }
}

/* ------------------------------
  - TODO: docstring
  ------------------------------ */
function connect_to_ros (data)
{
  var ros = new ROSLIB.Ros({ url: 'ws://' + data.address + ':' + data.port });

  ros.on('error', (error) => {
    show_snackbar('Couldn\'t connect to ros, check ip address or port number');
    console.log('connection error: ', error);
  }); // on error

  ros.on('close', () => {
    show_snackbar('Connection closed');
    console.log('connection closed');
  }); // on close

  ros.on('connection', () => {
    // inizialize new page
    build_app_page(data.address, data.port);

    // fill the global variable with partially applied method on ros

    call_service = call_service_builder(ros);

    get_services_1_list = get_services_1_list_builder(ros);
    get_services_2_list = get_services_2_list_builder();
    services_1_listener = services_1_listener_builder();
    services_2_listener = services_2_listener_builder(ros);

    get_launch_1_list = get_launch_1_list_builder(ros);
    get_launch_2_list = get_launch_2_list_builder();
    launch_1_listener = launch_1_listener_builder();
    launch_2_listener = launch_2_listener_builder(ros);

    get_wnodes_1_list = get_wnodes_1_list_builder();
    wnodes_1_listener = wnodes_1_listener_builder();

    // setup subscription for rosout
    rosout_subscription(ros);
  }); // on connection
}

/* ------------------------------------------------------------------------------------------
  - Services section
  ------------------------------------------------------------------------------------------ */
function update_services ()
{
  clear_services();
  toggle_visibility('services', 'block');
  toggle_visibility('services_1', 'block');

  get_services_1_list();
}

function get_services_1_list_builder (ros)
{
  return function ()
  {
    // get all running nodes on remote host
    ros.getNodes((data) => {
      update_list(
        // services_1 -> ul
        document.getElementById('services_1').children[1],
        data,
        services_1_listener_builder()
      );
    });
  }
}

function get_services_2_list_builder ()
{
  return function (node) {
    // get services list from current node
    call_service(
      '/service_list',
      'west_tools/ServiceList',
      { node: node },
      (result) => {
        update_list(
          // services_2 -> ul
          document.getElementById('services_2').children[1],
          result.service_list,
          services_2_listener
        );
      },
      (error) => {
        show_snackbar('Node : ' + node + ' does NOT contain available service!');
      }
    );
  }
}

function services_1_listener_builder ()
{
  return function (event) {
    // services_2 -> h2
    document.getElementById('services_2').children[0].innerHTML = 'Choice service from node : ' + event.target.innerHTML;
    // services_2 -> ul
    document.getElementById('services_2').children[1].innerHTML = '';

    toggle_visibility('services_1', 'none');
    toggle_visibility('services_2', 'block');

    get_services_2_list(event.target.innerHTML);
  }
}

function services_2_listener_builder (ros)
{
  return function (event) {
    toggle_visibility('services_2', 'none');
    // retrive service type and params by name
    ros.getServiceType(event.target.innerHTML, (type) => {
      ros.getServiceRequestDetails(type, (typeDetails) => {
        ros.getServiceResponseDetails(type, (responseDetails) => {
          toggle_visibility('services_3', 'block');
          build_services_3(event.target.innerHTML, typeDetails.typedefs[0], responseDetails.typedefs[0].fieldnames);
        });
      });
    });
  }
}

function build_services_3 (name, details, response)
{
  document.getElementById('services_4').children[1].setAttribute('fieldname', response[0]);
  let services_3 = document.getElementById('services_3');
  // services_3 -> h2
  services_3.children[0].innerHTML = 'Service name : ' + name + '<br> Service type : ' + details.type;

  let form = document.getElementById('services_3_form');


  form.setAttribute('service_name', name);
  form.setAttribute('service_details', details.type);

  let span = form.children[0];
  span.innerHTML = '';

  for (let i = 0; i < details.fieldnames.length; i++)
  {
    let label = document.createElement('label');
    let input = document.createElement('input');
    
    label.innerHTML = details.fieldnames[i];

    input.setAttribute('name', details.fieldnames[i]);
    input.setAttribute('class', 'w3-input w3-hover-cyan');
    input.setAttribute('placeholder', details.fieldtypes[i]);
    input.setAttribute('required', true);

    // set correct type for input
    if (details.fieldtypes[i] === 'string')
    {
      input.setAttribute('type', 'text');
    }
    else if (details.fieldtypes[i] === 'bool')
    {
      input.setAttribute('type', 'checkbox');
    }
    else
    {
      input.setAttribute('type', 'number');
      input.setAttribute('step', 'any');
    }

    span.appendChild(label);
    span.appendChild(input);
  }
}

function validate_services_3 ()
{
  let form = document.getElementById('services_3_form');
  // validate form!!
  let param = {};

  // subtract one to form to avoid count last once buttons
  for (let i = 0; i < form.elements.length; i++)
  {
    // if current element is an input tag
    if (isNaN(form.elements[i].value))
      param[form.elements[i].name] = form.elements[i].value;
    else
      param[form.elements[i].name] = Number(form.elements[i].value);
  }

  // call service
  call_service(
    form.getAttribute('service_name'),
    form.getAttribute('service_type'),
    param,
    build_services_4_success,
    build_services_4_error
  );

  toggle_visibility('services_3', 'none')
  toggle_visibility('services_4', 'block')
  // call service!! callback success and error
}

function build_services_4_success (result)
{
  // service_4 -> h2
  document.getElementById('services_4').children[0].innerHTML = 'Result :';
  // service_4 -> ul
  let ul = document.getElementById('services_4').children[1];
  let field = result[ul.getAttribute('fieldname')];

  let p = document.createElement('p');

  switch (typeof field)
  {
    case 'object':
    {
      for (let i = 0; i < field.length; i++)
      {
        let p = document.createElement('p')
        p.innerHTML = (i + 1) + ' - [' + field[i].toString() + ']';
        ul.appendChild(p);
      }
    } break;

    case 'undefined':
    {
      p.innerHTML = 'undefined';
      ul.appendChild(p);
    } break;

    case 'boolean':
    {
      p.innerHTML = 'boolean : ' + field;
      ul.appendChild(p);
    } break;

    default:
    {
      p.innerHTML = field.toString();
      ul.appendChild(p);
    }
  }
}

function build_services_4_error (error)
{
  // service_4 -> h2
  document.getElementById('services_4').children[0].innerHTML = 'Error : service NOT called';
  document.getElementById('services_4').children[1].innerHTML = error;
}

function clear_services ()
{
  toggle_visibility('services', 'none');
  toggle_visibility('services_1', 'none');
  toggle_visibility('services_2', 'none');
  toggle_visibility('services_3', 'none');
  toggle_visibility('services_4', 'none');

  // services_1 -> ul
  document.getElementById('services_1').children[1].innerHTML = '';

  // services_2 -> h2
  document.getElementById('services_2').children[0].innerHTML = '';
  // services_2 -> ul
  document.getElementById('services_2').children[1].innerHTML = '';

  // services_3 -> h2
  document.getElementById('services_3').children[0].innerHTML = '';
  // services_3_form -> span
  form = document.getElementById('services_3_form');
  form.children[0].innerHTML = '';
  form.removeAttribute('servicename');
  form.removeAttribute('servicetype');

  // services_4 -> h2
  document.getElementById('services_4').children[0].innerHTML = '';
  // services_4_form -> ul
  document.getElementById('services_4').children[1].innerHTML = '';
}

/* ------------------------------------------------------------------------------------------
  - Launch section
  ------------------------------------------------------------------------------------------ */

function update_launch ()
{
  clear_launch ();
  toggle_visibility('launch', 'block');
  toggle_visibility('launch_1', 'block');

  get_launch_1_list();
}

function get_launch_1_list_builder ()
{
  return function ()
  {
    call_service(
      '/pack_list',
      'west_tools/PackList',
      {},
      (result) => {
        update_list(
          // launch_1 -> ul
          document.getElementById('launch_1').children[1],
          result.pack_list,
          launch_1_listener_builder()
        )
      },
      (error) => {
        show_snackbar('Couldn\'t retrive available packages');
      }
    );
  }
}

function get_launch_2_list_builder ()
{
  return function (pack)
  {
    call_service(
      '/node_list',
      'west_tools/NodeList',
      { pack: pack },
      (result) => {
        // set attribute on form to retrive package name
        document.getElementById('launch_3_form').setAttribute('pack', pack);
        update_list(
          // services_2 -> ul
          document.getElementById('launch_2').children[1],
          result.node_list,
          launch_2_listener
        );
      },
      (error) => {
        show_snackbar('Package : ' + pack + ' does NOT contain available node!');
      }
    );
  }
}

function launch_1_listener_builder ()
{
  return function (event) {
    // launch_2 -> h2
    document.getElementById('launch_2').children[0].innerHTML = 'Choice node from package : ' + event.target.innerHTML;
    // launch_2 -> ul
    document.getElementById('launch_2').children[1].innerHTML = '';

    toggle_visibility('launch_1', 'none');
    toggle_visibility('launch_2', 'block');

    get_launch_2_list(event.target.innerHTML);
  }
}

function launch_2_listener_builder (ros)
{
  return function (event) {
    toggle_visibility('launch_2', 'none');
    toggle_visibility('launch_3', 'block');
    build_launch_3((document.getElementById('launch_3_form')).getAttribute('pack'), event.target.innerHTML);

    let form = document.getElementById('launch_3_form');
    form.setAttribute('node', event.target.innerHTML);
  }
}

function build_launch_3 (pack, node)
{
  let launch_3 = document.getElementById('launch_3');
  // launch_3 -> h2
  launch_3.children[0].innerHTML = 'Package name : ' + pack + '<br> Node name : ' + node;
}

function validate_launch_3 ()
{

  // call service to launch new node
  call_service(
    '/run_node',
    '/west_tools/RunNode',
    { pack: form.getAttribute('pack'), node: form.getAttribute('node') },
    build_launch_4_success,
    build_launch_4_error
  );

  toggle_visibility('launch_3', 'none')
  toggle_visibility('launch_4', 'block')
}

function build_launch_4_success (result)
{
  // launch_4 -> h2
  document.getElementById('launch_4').children[0].innerHTML = 'Node launched successfully';
  document.getElementById('launch_4').children[1].innerHTML = result;
}

function build_launch_4_error (error)
{
  // launch_4 -> h2
  document.getElementById('launch_4').children[0].innerHTML = 'Error : node NOT launched';
  document.getElementById('launch_4').children[1].innerHTML = error;
}

function clear_launch ()
{
  toggle_visibility('launch', 'none');
  toggle_visibility('launch_1', 'none');
  toggle_visibility('launch_2', 'none');
  toggle_visibility('launch_3', 'none');
  toggle_visibility('launch_4', 'none');

  // launch_1 -> ul
  document.getElementById('launch_1').children[1].innerHTML = '';

  // launch_2 -> h2
  document.getElementById('launch_2').children[0].innerHTML = '';
  // launch_2 -> ul
  document.getElementById('launch_2').children[1].innerHTML = '';

  // launch_3 -> h2
  document.getElementById('launch_3').children[0].innerHTML = '';
  // launch_3_form -> span
  form = document.getElementById('launch_3_form');
  form.children[0].innerHTML = '';
  form.removeAttribute('pack');
  form.removeAttribute('node');

  // launch_4 -> h2
  document.getElementById('launch_4').children[0].innerHTML = '';
  // launch_4_form -> ul
  document.getElementById('launch_4').children[1].innerHTML = '';
}

/* ------------------------------------------------------------------------------------------
  - West Nodes section
  ------------------------------------------------------------------------------------------ */

function update_wnodes ()
{
  clear_wnodes ();
  toggle_visibility('wnodes', 'block');
  toggle_visibility('wnodes_1', 'block');

  get_wnodes_1_list();
}

function get_wnodes_1_list_builder ()
{
  return function ()
  {
    call_service(
      '/wnode_list',
      'west_tools/WNodeList',
      {},
      (result) => {
        update_list(
          // wnodes_1 -> ul
          document.getElementById('wnodes_1').children[1],
          result.wnode_list,
          wnodes_1_listener_builder()
        )
      },
      (error) => {
        show_snackbar('Couldn\'t retrive wnodes');
      }
    );
  }
}

function validate_wnodes_2 (input)
{
  let form = document.getElementById('wnodes_2_form');
  // if we haven't an input string, take it from form
  if (input == null)
    input = form.elements[0].value;
  // call service to launch new node
  call_service(
    '/wnode_input',
    '/west_tools/WNodeInput',
    { node: form.getAttribute('wnode'), text: input },
    (result) => { form.elements[0].value = ''; show_snackbar('Input string sent'); },
    (error) => { show_snackbar('Input string NOT sent'); }
  );
}

function wnodes_1_listener_builder ()
{
  return function (event) {
    // wnodes_2 -> h2
    document.getElementById('wnodes_2').children[0].innerHTML = 'Wnode selected : ' + event.target.innerHTML;

    toggle_visibility('wnodes_1', 'none');
    toggle_visibility('wnodes_2', 'block');

    (document.getElementById('wnodes_2_form')).setAttribute('wnode', event.target.innerHTML);
  }
}

function clear_wnodes ()
{
  toggle_visibility('wnodes', 'none');
  toggle_visibility('wnodes_1', 'none');
  toggle_visibility('wnodes_2', 'none');

  // wnodes_1 -> ul
  document.getElementById('wnodes_1').children[1].innerHTML = '';

  // wnodes_2 -> h2
  document.getElementById('wnodes_2').children[0].innerHTML = '';

  (document.getElementById('wnodes_2_form')).removeAttribute('wnode');
}

/* ------------------------------
  - Kill node passed as parameter
  - Call service '/kill_node' provided by west_tools rosnode
  ------------------------------ */
function kill_wnodes_listener ()
{
  toggle_visibility('wnodes_2');
  // call service
  call_service(
    '/kill_node',
    '/west_tools/KillNode',
    { node: (document.getElementById('wnodes_2_form')).getAttribute('wnode') },
    (result) => {
      show_snackbar('Node killed successfully');
      clear_wnodes();
    },
    (error) => {
      show_snackbar('Node NOT killed!');
    }
  );
}

/* ------------------------------------------------------------------------------------------
  - Logs section
  ------------------------------------------------------------------------------------------ */

/* ------------------------------
  - Subscrive to rosout topic and update logs section
  each time it is published a new topic
  ------------------------------ */
function rosout_subscription (ros)
{
  let rosout = new ROSLIB.Topic({
      ros: ros,
      name: '/rosout',
      messageType: 'rosgraph_msgs/Log'
    });
    
  rosout.subscribe( (message) => {
    // create a new subscription entry
    let li = document.createElement('li');
    li.setAttribute('class', 'w3-bar');

    // left: topic name
    li.innerHTML = '<span style="float: left;"><b>/rosout</b>:</span>';
    // right: message
    li.innerHTML += '</span style="float: right;">' + message.msg + '</span>';

    // append to list (first child of logs section)
    document.getElementById('logs').children[1].appendChild(li);
  });
}
