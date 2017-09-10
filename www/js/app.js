/* ------------------------------
  - To avoid exposing the ros variable, we maintain a cache that contains
  a number of features. These methods are partially applied, that is, they
  are functions that use the ros variable but do not require it as an argument.
  ------------------------------ */
  var cache = {
    call_service: undefined,
    launch_node: undefined,
    launch_service: undefined,
    update_packages: undefined,
    update_nodes: undefined
  };

/* ------------------------------------------------------------------------------------------
  - Partially applied method on @ros, return a function with which you can call a service.
  ------------------------------------------------------------------------------------------ */

/* ------------------------------
  -Result function parameters are :
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

/* ------------------------------
  - Callback of service click
  - TODO: docstring
  ------------------------------ */
function launch_node (event)
{
  // call service to launch new node
  cache.call_service(
    '/run_node',
    '/west_tools/RunNode',
    {
      pack: event.target.parentNode.parentNode.getAttribute('name'),
      node: event.target.innerHTML
    },
    (result) => {
      show_snackbar('Node : ' + event.target.innerHTML + ' launched successfully');
    },
    (error) => {
      show_snackbar('Node : ' + event.target.innerHTML + ' node NOT launched!');
      console.log(error);
    }
  );

  // finally, after one second, refresh app page
  refresh_page(1000);
}

/* ------------------------------
  - Callback builder of service click, return a function like 'function (event) {}'
  - TODO: docstring
  ------------------------------ */
function launch_service_builder (ros)
{
  return function (event)
  {
    // retrive service type and params by name
    ros.getServiceType(event.target.innerHTML, (type) => {
      ros.getServiceRequestDetails(type, (typeDetails) => {
        ros.getServiceResponseDetails(type, (responseDetails) => {
          clear_param_section();
          toggle_visibility('controls', 'none');
          build_param_section(event.target.innerHTML, typeDetails.typedefs[0], responseDetails.typedefs[0].fieldnames);
        });
      });
    });
  };
}

/* ------------------------------
  - TODO: docstring
  ------------------------------ */
function update_available_packages ()
{
  packages = undefined;
  document.getElementById('packages').children[0].innerHTML = '';

  cache.call_service(
    '/pack_list',
    'west_tools/PackList',
    {},
    (result) => {
        packages = [];
        for (let i = 0; i < result.pack_list.length; i++)
        {
          packages.push(
          {
            name: result.pack_list[i],
            nodes: undefined
          });
        }
        // manually trigger packages list view update
        if (result.pack_list.length >= 1 && result.pack_list[0] !== '')
        {
          update_list(
            document.getElementById('packages').children[0],
            packages,
            list_packages_listener
          );
        }
    },
    (error) => {
        show_snackbar('Couldn\'t retrive available packages');
        console.log('pack_list:  ' + error);
    }
  );
}

/* ------------------------------
  - TODO: docstring
  ------------------------------ */
function update_available_nodes_builder (ros)
{
  return function () {
    // get all running nodes on remote host
    nodes = undefined;
    document.getElementById('running_list').children[0].innerHTML = '';
  
    // get all running nodes on remote host
    ros.getNodes((data) => {
        nodes = [];
  
        for (let i = 0; i < data.length; i++)
        {
          nodes.push(
          {
            name: data[i],
            services: undefined
          });
        }
        // manually trigger nodes list view update
        if (data.length >= 1 && data[0] !== '')
        { 
          update_list(
            document.getElementById('running_list').children[0],
            nodes,
            list_nodes_listener,
            kill_node_listener
          );
        }
    });
  }
}


/* ------------------------------------------------------------------------------------------
  - Lists updater and relative listeners
  ------------------------------------------------------------------------------------------ */

/* ------------------------------
  - Create primary list
  ------------------------------ */
function update_list (parent, list, listener, kill_listener)
{
  for (let i = 0; i < list.length; i++)
  {
    let li = document.createElement('li');
    let h4 = document.createElement('h4');

    // for each element of list inizialize flag to perform the operation one time
    list[i].executed = false;

    li.setAttribute('class', 'w3-display-container w3-bar w3-hover-cyan');
    h4.innerHTML = list[i].name;

    li.addEventListener('click', (event) => { listener(event.target.parentNode, list[i]); } );

    li.appendChild(h4);
    
    if (kill_listener)
    {
      let kill = document.createElement('i');
      //kill.innerHTML = '&times';
      kill.setAttribute('class', 'fa fa-close w3-large w3-display-topright');
      kill.addEventListener('click', (event) => { kill_listener(list[i]); } );
      li.appendChild(kill);
    }
    parent.appendChild(li);
  }
}

/* ------------------------------
  - Create drop down list
  ------------------------------ */
function update_sublist (curr, parent, name, id, listener)
{
  // don't create sublists if no nodes are available
  if (curr[0] === "") return;

  // build sublist from cache
  let sub = document.createElement('ul');
    // append sublist to first level list
  parent.appendChild(sub);
  sub.id = id;
  sub.setAttribute('name', name);
  sub.setAttribute('class','w3-ul w3-card-4');
  sub.style.display = 'block';

  for (let i = 0; i < curr.length; i++)
  {
    let sub_el = document.createElement('li');

    sub_el.setAttribute('class', 'w3-bar w3-hover-white');
    let h6 = document.createElement('h6');

    h6.innerHTML = curr[i];
    // for each element set onclick event
    sub_el.addEventListener('click', listener);
    sub_el.appendChild(h6);
    sub.appendChild(sub_el);
  }
  // append sublist to first level list
  //parent.appendChild(sub);
}

/* ------------------------------
  - Retrive, with service call, all nodes available for the pack
  ------------------------------ */
function list_packages_listener (parent, package)
{
  // check if operation has alrwedy been performed
  if (package.executed || package.nodes !== undefined)
  {
    toggle_visibility(package.name + '_nodes');
  }
  else
  {
    // switch flag to true and perform for the first (and last) time the operation
    package.executed = true;
    // get nodes list for current package
    cache.call_service(
      '/node_list',
      'west_tools/NodeList',
      { pack: package.name },
      (result) => {
        package.nodes = [];
        for (let j = 0; j < result.node_list.length; j++)
        {
          package.nodes.push(result.node_list[j]);
        }
        // trigger list view update
        if (result.node_list.length >= 1 && result.node_list[0] !== '')
        { 
          update_sublist(
            package.nodes,
            parent,
            package.name,
            package.name + '_nodes',
            cache.launch_node
          );
        }
      },
      (error) => {
        show_snackbar('Package : ' + package.name + ' does NOT contain available node!');
        console.log('node_list:  ' + error);
      }
    );
  }
}

/* ------------------------------
  - Retrive, with service call, all services available for the node
  ------------------------------ */
function list_nodes_listener (parent, node)
{
  // check if operation has alrwedy been performed
  if (node.executed || node.services !== undefined)
  {
    toggle_visibility(node.name + '_services');
  }
  else
  {
    // switch flag to true and perform for the first (and last) time the operation
    node.executed = true;
    // get services list from current node
    cache.call_service(
      '/service_list',
      'west_tools/ServiceList',
      { node: node.name },
      (result) => {
        node.services = [];
        for (let j = 0; j < result.service_list.length; j++)
        {
          node.services.push(result.service_list[j]);
        }
        //trigger list view update
        if (result.service_list.length >= 1 && result.service_list[0] !== '')
        {
          update_sublist(
            node.services,
            parent,
            node.name,
            node.name + '_services',
            cache.launch_service
          );
        } 
      },
      (error) => {
        show_snackbar('Node : ' + node.name + ' does NOT contain available service!');
        console.log('service_list:  ' + error);
      }
    );
  }
}

/* ------------------------------
  - TODO: docstring
  ------------------------------ */
function kill_node_listener (node)
{
  // call service
  cache.call_service(
    '/kill_node',
    '/west_tools/KillNode',
    { node: node.name },
    (result) => {
      show_snackbar('Node : ' + node. name + ' killed successfully');
    },
    (error) => {
      show_snackbar('Node : ' + node. name + ' NOT killed!');
      console.log(error);
    }
  );

  // finally, after one second, refresh app page
  refresh_page(1000);
}


/* ------------------------------------------------------------------------------------------
  - Param section utility : build, validate, clear
  ------------------------------------------------------------------------------------------ */

/* ------------------------------
  - Build service call parameters form with request details
  ------------------------------ */
function build_param_section (name, details, response)
{
  document.getElementById('result').children[1].setAttribute('fieldname', response[0]);

  let param_section = document.getElementById('param_section');
  // param_section -> h2
  let h2 = param_section.children[0];

  let form = document.getElementById('param_form');
  // form -> ul
  let ul = form.children[0];

  h2.innerHTML = 'Service name : ' + name + '<br>Service type : ' + details.type;

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
    else
    {
      input.setAttribute('type', 'number');
      input.setAttribute('step', 'any');
    }

    li.appendChild(h3);
    li.appendChild(input);
    ul.appendChild(li);
  }

  toggle_visibility('param_section', 'block');
  toggle_visibility('running_list', 'none');
  toggle_visibility('back_service', 'inline-block');
}

/* ------------------------------
  - Check if param form input is valid. If it is, try to call relative service
  ------------------------------ */
function validate_param_section (form)
{
  // form -> ul
  let ul = document.getElementById('param_form').children[0];
  let param = {};

  // subtract one to form to avoid count last once buttons
  for (let i = 0; i < form.elements.length - 1; i++)
  {
    // if current element is an input tag
    if (isNaN(form.elements[i].value))
      param[form.elements[i].name] = form.elements[i].value;
    else
      param[form.elements[i].name] = Number(form.elements[i].value);
  }

  // call service
  cache.call_service(
    form.getAttribute('service_name'),
    form.getAttribute('service_type'),
    param,
    (result) => {
      let div = document.getElementById('result').children[1];
      let field = result[div.getAttribute('fieldname')];
      
      let p = document.createElement('p');

      switch (typeof field)
      {
        case 'object':
        {
          for (let i = 0; i < field.length; i++)
          {
            let p = document.createElement('p')
            p.innerHTML = (i + 1) + ' - [' + field[i].toString() + ']';
            div.appendChild(p);
          }
        } break;

        case 'undefined':
        {
          p.innerHTML = 'undefined';
          div.appendChild(p);
        } break;

        case 'boolean':
        {
          p.innerHTML = 'boolean : ' + field;
          div.appendChild(p);
        } break;

        default:
        {
          p.innerHTML = field.toString();
          div.appendChild(p);
        }
      }

      toggle_visibility('result');
    },
    (error) => {
      show_snackbar('Service : ' + form.getAttribute('service_name') + ' NOT called!');
      console.log(error);
    }
  );

  // clear service param modal
  form.removeAttribute('servicename');
  form.removeAttribute('servicetype');
  
  toggle_visibility('param_form', 'none');
  toggle_visibility('back_service', 'none');  
}

/* ------------------------------
  - Cancel service request
  ------------------------------ */
function clear_param_section ()
{
  toggle_visibility('param_section', 'none');
  toggle_visibility('result', 'none');
  toggle_visibility('param_form', 'block');
  toggle_visibility('controls', 'block');
  toggle_visibility('back_service', 'none');
  toggle_visibility('running_list', 'block');
  
  // param_section -> header
  document.getElementById('param_section').children[0].innerHTML = '';
  // form -> ul
  document.getElementById('param_form').children[0].innerHTML = '';
  // result
  document.getElementById('result').children[1].innerHTML = '';
}


/* ------------------------------------------------------------------------------------------
  - Page managment and effects
  ------------------------------------------------------------------------------------------ */

/* ------------------------------
  - Set or toggle visibility to the tag identified by id
  ------------------------------ */
function toggle_visibility (id, status)
{
  let element = document.getElementById(id);

  if (!element)
    return;

  // if given, set new display value
  if (status)
  {
    element.style.display = status;
    return;
  }

  // if not, flip current value between block and none
  if (!(element.style.display) || element.style.display === 'none')
    element.style.display = 'block';
  else if (element.style.display === 'block' || element.style.display === 'inline-block')
    element.style.display = 'none';
}

/* ------------------------------
  - Without params the section with id as 'running', 'logs' and 'packages'
  are hidden, and each controls button are unselected
  (with class 'w3-cyan' is consider selected)
  - With @id set on 'block' the display value to tag identified by id
  - With @button set button as selected
  ------------------------------ */
function switch_controls(id, button)
{
  toggle_visibility('running', 'none');
  toggle_visibility('logs', 'none');
  toggle_visibility('packages', 'none');
  
  controls = document.getElementById('controls').children;
  for (let i = 0; i < controls.length; i++)
    controls[i].classList.remove('w3-cyan');
  
  // With @id
  if(id)
    toggle_visibility(id, 'block');

  // With @button
  if (button)
    button.classList.add('w3-cyan');
}

/* ------------------------------
  - Hide all section except control and update list of available packages and running nodes
  ------------------------------ */
function refresh_page (timeout)
{
  // animate refresh button 
  refresh = document.getElementById('refresh');
  refresh.classList.add('w3-spin');
  //refresh.setAttribute('class', 'glyphicon glyphicon-refresh w3-xlarge w3-spin');

  clear_param_section();
  switch_controls('controls');

  setTimeout( () => {
    cache.update_packages();
    cache.update_nodes();

    // stop animation
    refresh.classList.remove('w3-spin');
    //refresh.setAttribute('class', 'glyphicon glyphicon-refresh w3-xlarge');
  }, timeout);
}

/* ------------------------------
  - Simple snackbar with given message
  ------------------------------ */
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

/* ------------------------------
  - Create a simple html header and show the new page
  ------------------------------ */
function build_app_page (address, port)
{
  let header = document.getElementsByTagName('header')[0];
  console.log(header);

  // set title and some info about remote host
  header.children[0].innerHTML = 'West'
  header.children[1].innerHTML = 'connected to <b>' + address + '</b> on port <b>' + port + '</b>';

  // hide connection form
  toggle_visibility('connection', 'none');
  // show controls on connection
  toggle_visibility('app_page', 'block');
  toggle_visibility('controls', 'block');
  toggle_visibility('refresh', 'inline-block');
}

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

  if (conn_data.address && conn_data.port)
    connect_to_ros(conn_data);
}

/* ------------------------------------------------------------------------------------------
  ------------------------------------------------------------------------------------------ */


/* ------------------------------
  - Subscrive to rosout topic
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

    // left: topic name
    li.innerHTML = '<span style="float: left;"><b>/rosout</b>:</span>';
    // right: message
    li.innerHTML += '</span style="float: right;">' + message.msg + '</span>';

    // append to list (first child of logs section)
    document.getElementById('logs').children[0].appendChild(li);
  });
}

/* ------------------------------
  - TODO: docstring
  ------------------------------ */
function connect_to_ros (data)
{
  var ros = new ROSLIB.Ros({
    url: 'ws://' + data.address + ':' + data.port
  });

  ros.on('error', (error) => {
    show_snackbar('Couldn\'t connect to ros, check ip address or port number');
    console.log('connection error: ', error);
  }); // on error

  ros.on('close', () => {
    console.log('connection closed');
  }); // on close

  ros.on('connection', () => {
    // show connection page
    build_app_page( data.address, data.port);

    // fill the cache with partially applied method on ros
    cache.call_service = call_service_builder(ros);
    cache.launch_node = launch_node;
    cache.launch_service = launch_service_builder(ros);
    cache.update_packages = update_available_packages;
    cache.update_nodes = update_available_nodes_builder(ros);

    // get available packages
    cache.update_packages();

    // setup subscription for rosout
    rosout_subscription(ros);

    // get all running nodes on remote host
    cache.update_nodes();
  }); // on connection
}

/* ------------------------------
  - TODO: docstring
  ------------------------------ */
window.onload = function ()
{
  // setup initial page state
  toggle_visibility('connection', 'block');
  // hide all section of app
  toggle_visibility('app_page', 'none');
  toggle_visibility('controls', 'none');
  toggle_visibility('running', 'none');
  toggle_visibility('logs', 'none');
  toggle_visibility('packages', 'none');
  // hide button
  toggle_visibility('refresh', 'none');
  toggle_visibility('back_service', 'none');

  // id running
  toggle_visibility('running_list', 'block');
  toggle_visibility('param_section', 'none');
  toggle_visibility('result', 'none');
}
