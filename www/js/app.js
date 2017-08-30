// cache object to store remote calls output
var cache = { ros: undefined, packages: undefined, nodes: undefined };

// set or toggle visibility to the section identified by id
function toggle_visibility (id, status)
{
  let element = document.getElementById(id);

  if (!element)
    return;

  // if given, set new display value
  if (status) {
    element.style.display = status;
    return;
  }

  // if not, flip current value between block and none
  if (!(element.style.display) || element.style.display === 'none')
    element.style.display = 'block';
  else if (element.style.display === 'block' || element.style.display === 'inline-block')
    element.style.display = 'none';
}

// set visibility to one section at a time
function switch_visibility (id)
{
  // hide all section
  toggle_visibility('running', 'none');
  toggle_visibility('logs', 'none');
  toggle_visibility('packages', 'none');
  toggle_visibility('param_section', 'none');
  // display only selected section
  toggle_visibility(id, 'block');
}

// simple snackbar with given message
function show_snackbar (message)
{
  // get the snackbar div
  snackbar = document.getElementById('snackbar');
  snackbar.innerHTML = message;
  // add the 'show' class to div
  snackbar.className = 'show';
  // after 5 seconds, remove the show class from div
  setTimeout( () => { snackbar.className = ''; }, 5000);
}

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
    update_header(conn_data.address, conn_data.port);
    connect_to_ros(conn_data);
  }
}

// build service call parameters form with request details
function build_param_section (name, details)
{
  let param_section = document.getElementById('param_section');
  // param_section -> header
  let header = param_section.children[0];

  let form = document.getElementById('param_form');
  // form -> ul
  let ul = form.children[0];

  header.innerHTML = 'Service name : ' + name + '<br>Service type : ' + details.type;

  form.setAttribute('service_name', name);
  form.setAttribute('service_details', details.type);

  for (let i = 0; i < details.fieldnames.length; i++)
  {
    let li = document.createElement('li');
    let h3 = document.createElement('h3');
    let input = document.createElement('input');
    
    h3.innerHTML = details.fieldnames[i];

    input.setAttribute('name', details.fieldnames[i]);
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
    else {
      input.setAttribute('type', 'number');
      input.setAttribute('step', 'any');
    }

    li.appendChild(h3);
    li.appendChild(input);
    ul.appendChild(li);
  }

  switch_visibility('param_section');
}

// Check if param form input is valid. If it is, try to call relative service
function validate_param_section (form)
{
  // form -> ul
  let ul = document.getElementById('param_form').children[0];
  let param = {};

  // subtract two to form to avoid count last two buttons
  for (let i = 0; i < form.elements.length - 2; i++)
  {
    // if current element is an input tag
    if (isNaN(form.elements[i].value))
      param[form.elements[i].name] = form.elements[i].value;
    else
      param[form.elements[i].name] = Number(form.elements[i].value);
  }

  // call service
  call_service(
    cache.ros,
    form.getAttribute('service_name'),
    form.getAttribute('service_type'),
    param,
    (result) => {
      show_snackbar('service called successfully!');
      console.log(result);
    },
    (error) => {
      show_snackbar('service NOT called!');
      console.log(error);
    }
  );

  // clear service param modal
  form.removeAttribute('servicename');
  form.removeAttribute('servicetype');
  clear_param_section();
}

// cancel service request
function clear_param_section ()
{
  toggle_visibility('param_section', 'none');
  // param_section -> header
  document.getElementById('param_section').children[0] = '';
  // form -> ul
  document.getElementById('param_form').children[0].innerHTML = '';
}

// TODO: docstring
function call_service (ros, name, type, params, success_cb, error_cb)
{
  let srv = new ROSLIB.Service({
    ros: ros,
    name: name,
    serviceType: type
  });

  let request = new ROSLIB.ServiceRequest(params);

  srv.callService(request, success_cb, error_cb);
}

// TODO: launch new node from west backend
function launch_node (event)
{
  console.log('launching ' + event.target.innerHTML + ' ...');
}

// callback of service click
function launch_service_builder (ros)
{
  return function (event)
  {
    console.log('launching ' + event.target.innerHTML + ' service ...');
    // retrive service type and params by name
    ros.getServiceType(event.target.innerHTML, (type) => {
      ros.getServiceRequestDetails(type, (typeDetails) => {
        clear_param_section();
        build_param_section(event.target.innerHTML, typeDetails.typedefs[0]);    
      });
    });
  };
}

// create a simple html header
function update_header (address, port)
{
  let header = document.getElementsByTagName('header')[0];

  // set title and some info about remote host
  header.innerHTML = '<h2>west</h2>'
  header.innerHTML += '<p>connected to <b>' + address +
                     '</b> on port <b>' + port + '</b></p>';

  // hide connection form
  toggle_visibility('connection', 'none');
}

// create drop down list
function update_sublist (curr, parent, id, fun)
{
  // don't create sublists if no nodes are available
  if (curr[0] === "") return;

  // build sublist from cache
  let sub = document.createElement('ul');
  sub.id = id;
  sub.style.display = 'inline-block';

  for (let i = 0; i < curr.length; i++)
  {
    let sub_el = document.createElement('li');
    let h5 = document.createElement('h5');

    h5.innerHTML = curr[i];
    // for each element set onclick event
    h5.addEventListener('click', fun);
    sub_el.appendChild(h5);
    sub.appendChild(sub_el);
  }
  // append sublist to first level list
  parent.appendChild(sub);
}

function update_available_packages (ros)
{
  if (cache && cache.packages !== undefined)
  {
    for (let i = 0; i < cache.packages.length; i++)
    {
      let li = document.createElement('li');
      let h4 = document.createElement('h4');

      h4.innerHTML = cache.packages[i].name;

      h4.addEventListener('click', function (event) {
        let parent = event.target.parentNode;

        if (cache.packages[i].nodes !== undefined) {
          toggle_visibility(cache.packages[i].name + '_nodes');
        }
        else
        {
          // get nodes list for current package
          call_service(ros,
            '/node_list', 'west_tools/NodeList',
            { pack: cache.packages[i].name },
            (result) => {
              cache.packages[i].nodes = [];
              for (let j = 0; j < result.node_list.length; j++) {
                cache.packages[i].nodes.push(result.node_list[j]);
              }
              // trigger list view update
              if (result.node_list.length >= 1 && result.node_list[0] !== '')
                update_sublist(cache.packages[i].nodes, parent, cache.packages[i].name + '_nodes', launch_node);
            },
            (error) => {
              console.log('node_list:  ' + error);
            }
          );
        }
      });

      li.appendChild(h4);
      document.getElementById('packages').children[0].appendChild(li);
    }
  }
}

function update_available_nodes (ros)
{
  if (cache && cache.nodes !== undefined)
  {
    for (let i = 0; i < cache.nodes.length; i++)
    {
      let li = document.createElement('li');
      let h4 = document.createElement('h4');

      h4.innerHTML = cache.nodes[i].name;

      h4.addEventListener('click', function (event) {
        let parent = event.target.parentNode;

        if (cache.nodes[i].services !== undefined)
        {
          toggle_visibility(cache.nodes[i].name + '_services');
        }
        else
        {
          // get services list from current node
          call_service(ros,
            '/service_list', 'west_tools/ServiceList',
            { node: cache.nodes[i].name },
            (result) => {
              cache.nodes[i].services = [];
              for (let j = 0; j < result.service_list.length; j++) {
                cache.nodes[i].services.push(result.service_list[j]);
              }
              //trigger list view update
              if (result.service_list.length >= 1 && result.service_list[0] !== '')
                update_sublist(
                  cache.nodes[i].services,
                  parent,
                  cache.nodes[i].name + '_services',
                  launch_service_builder(ros)
                );
            },
            (error) => {
              console.log('service_list:  ' + error);
            }
          );
        }
      });

      li.appendChild(h4);
      document.getElementById('running').children[0].appendChild(li);
    }
  }
}

function connect_to_ros (data)
{
  var ros = new ROSLIB.Ros({
    url: 'ws://' + data.address + ':' + data.port
  });
  // store ros variable in cache
  cache.ros = ros;

  ros.on('error', (error) => {
    console.log('connection error: ', error);
  }); // on error

  ros.on('close', () => {
    console.log('connection closed');
  }); // on close

  ros.on('connection', () => {
    // show controls on connection
    toggle_visibility('controls', 'block');

    // get available packages if not already stored in cache
    if (cache.packages === undefined)
    {
      call_service(
        ros, '/pack_list', 'west_tools/PackList', {},
        (result) =>{
          cache.packages = [];
          for (let i = 0; i < result.pack_list.length; i++)
          {
            cache.packages.push({
              name: result.pack_list[i],
              nodes: undefined
            });
          }
          // manually trigger packages list view update
          if (result.pack_list.length >= 1 && result.pack_list[0] !== '')
            update_available_packages(ros);
        },
        (error) =>{
          console.log('pack_list:  ' + error);
        }
      );
    }

    // setup subscription for rosout
    let rosout = new ROSLIB.Topic({
      ros: ros,
      name: '/rosout',
      messageType: 'rosgraph_msgs/Log'
    });
    rosout.subscribe(function (message)
    {
      // create a new subscription entry
      let li = document.createElement('li');

      // left: topic name
      li.innerHTML = '<span style="float: left;"><b>/rosout</b>:</span>';
      // right: message
      li.innerHTML += '</span style="float: right;">' + message.msg + '</span>';

      // append to list (first child of logs section)
      document.getElementById('logs').children[0].appendChild(li);
    });

    // get all running nodes on remote host
    if (cache.nodes === undefined)
    {
      // get all running nodes on remote host
      ros.getNodes((data) => {
        cache.nodes = [];

        for (let i = 0; i < data.length; i++)
        {
          cache.nodes.push({
            name: data[i],
            services: undefined
          });
        }
        // manually trigger nodes list view update
        if (data.length >= 1 && data[0] !== '')
          update_available_nodes(ros);
      });
    }
  }); // on connection
}

window.onload = function ()
{
  // setup initial page state
  toggle_visibility('connection', 'block');
  toggle_visibility('controls', 'none');
  toggle_visibility('running', 'none');
  toggle_visibility('logs', 'none');
  toggle_visibility('packages', 'none');
  toggle_visibility('param_section', 'none');
}
