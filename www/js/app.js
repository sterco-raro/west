var validate_params = undefined;

// Check if form input is valid. If it is, try to connect to ros
function validate (form)
{
  let conn_data = {};

  for (let i = 0; i < form.elements.length; i++)
  {
    if (form.elements[i].name && form.elements[i].value)
      conn_data[form.elements[i].name] = form.elements[i].value;
  }

  if (conn_data.address && conn_data.port)
  {
    update_view(conn_data.address, conn_data.port);
    connect_to_ros(conn_data);
  }
}

function validate_builder (ros)
{
  return function (form) {
    console.log('IMANEWFUNCTION');
    // check values for all input tags
    // the last element is a submit button, hence the length - 1
    let params = {};
    for (let i = 0; i < form.elements.length - 1; i++)
    {
      // if current element is an input tag
      if (isNaN(form.elements[i].value))
        params[form.elements[i].name] = form.elements[i].value;
      else
        params[form.elements[i].name] = Number(form.elements[i].value);
    }
    // hide modal
    document.getElementById('service_call_modal').style.display = 'none';
    // reset form
    document.getElementById('input_container').innerHTML = '';
    // actual service call
    call_service(ros, form.getAttribute('servicename'), form.getAttribute('servicetype'), params);
  };
}

/*
  TODO: docstring
*/
function call_service (ros, name, type, params, success_cb, error_cb)
{
  let srv = new ROSLIB.Service({
    ros: ros,
    name: name,
    serviceType: type
  });

  let request = new ROSLIB.ServiceRequest(params);

  srv.callService(request, (result) => {
    console.log('call_service success');
    console.log(result);
  }, (error) => {
    console.log('call_service failure');
    console.log(error);
  });
}

// build service call parameters form with request details
function build_params_form (name, details)
{
  let form = document.getElementById('service_form');
  let div = document.getElementById('input_container');

  // save service info inside form
  form.setAttribute('servicename', name);
  form.setAttribute('servicetype', details.type);

  // create an input tag for every service param
  // fieldnames[] and fieldtypes[] have same length 
  for (let i = 0; i < details.fieldnames.length; i++)
  {
    let label = document.createElement('h4');
    let input = document.createElement('input');

    // set label as param name
    label.innerHTML = details.fieldnames[i];
    // setup input field
    input.setAttribute('name', details.fieldnames[i]);
    input.setAttribute('placeholder', details.fieldtypes[i]);
    input.setAttribute('required', true);

    // set correct type for input
    let type = '';
    if (details.fieldtypes[i] === 'string')
      type = 'text';
    else if (details.fieldtypes[i] === 'bool')
      type = 'checkbox';
    else {
      type = 'number';
      input.setAttribute('step', 'any');
    }
    input.setAttribute('type', type);

    div.appendChild(label);
    div.appendChild(input);
  }
  // done creating form, show modal
  document.getElementById('service_call_modal').style.display = 'block';
}

// update header text and show controls
function update_view (address, port)
{
  let header = document.getElementsByTagName('header')[0];

  header.innerHTML = '<p>connected to <b>' + address +
                     '</b> on port <b>' + port + '</b></p>';

  // hide connection modal and show controls section
  document.getElementById('connection_modal').style.display = 'none';
  document.getElementById('controls').style.display = 'block';  
}

// try to connect to ros remote through websocket
function connect_to_ros (data)
{
  let ros = new ROSLIB.Ros({
    url: 'ws://' + data.address + ':' + data.port
  });

  // handle connection errors
  ros.on('error', (error) => {
    console.log('connection error: ', error);

    ros = undefined;
  });

  ros.on('close', () => {
    console.log('connection closed');

    ros = undefined;
  });

  ros.on('connection', () => {
    console.log('connection established');

    // build validate function for call_service params and attach it to the submit button
    validate_params = validate_builder(ros);
    document.getElementById('confirm_call', validate_params, false);

    // subscribe to a topic and update view with data
    ros.getServices((data) => {
      let list = document.getElementById('services');

      // click handler on services list
      list.addEventListener('click', function (event) {
        let target = event.target || event.srcElement;

        ros.getServiceType(target.innerHTML, (type) => {
          ros.getServiceRequestDetails(type, (typeDetails) => {
            // build input form with typeDetails
            build_params_form(target.innerHTML, typeDetails.typedefs[0]);
          }); // getServiceRequestDetails
        }); // getServiceType
      }, false);// onclick listener

      for (let i = 0; i < data.length; i++)
      {
        let li = document.createElement('li');
        let h5 = document.createElement('h5');

        h5.innerHTML = data[i];

        li.appendChild(h5);
        list.appendChild(li);
      }
    });

    ros.getNodes((data) => {
      let list = document.getElementById('nodes');
      for (let i = 0; i < data.length; i++)
      {
        let li = document.createElement('li');
        li.innerHTML = '<h5>' + data[i] + '<span style="float: right;">&times;</span></h5>';
        li.setAttribute('node_name', data[i]);
        list.appendChild(li);
      }
    });

    let rosout = new ROSLIB.Topic({
      ros: ros,
      name: '/rosout',
      messageType: 'rosgraph_msgs/Log'
    });

    rosout.subscribe(function (message)
    {
      let li = document.createElement('li');

      // left: topic name
      li.innerHTML = '<span style="float: left;"><b>/rosout</b>:</span>';
      // right: message
      li.innerHTML += '</span style="float: right;">' + message.msg + '</span>';
      
      document.getElementById('logs').appendChild(li);
    });
  });
}

// toggle section visibility
function show_section (name)
{
  let section = document.getElementById('section_' + name);
  let button = document.getElementById('button_' + name);

  // toggle section diplay style
  if (!(section.style.display) || section.style.display === 'none')
    section.style.display = 'block';
  else if (section.style.display === 'block')
    section.style.display = 'none';

  // toggle button class
  if (section.style.display === 'block')
    button.className = 'control-on';
  else
    button.className = 'control-off';
}

window.onload = function () {
  // show connection popup
  document.getElementById('connection_modal').style.display = 'block';
}
