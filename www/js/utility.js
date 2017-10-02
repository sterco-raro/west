
/* ------------------------------------------------------------------------------------------
  - General utility
  ------------------------------------------------------------------------------------------ */

/* ------------------------------
  - Create a simple html header and show the new page
  ------------------------------ */
function build_app_page (address, port)
{
  let header = document.getElementsByTagName('header')[0];

  // set title and some info about remote host
  // header -> h4
  header.children[1].innerHTML = 'connected to <b>' + address + '</b> on port <b>' + port + '</b> &nbsp;';

  // hide connection form
  toggle_visibility('connection', 'none');
  // show controls on connection
  toggle_visibility('refresh', 'inline-block');
  toggle_visibility('controls', 'block');
  toggle_visibility('app_page', 'block');
}

/* ------------------------------
  - For each element of list create a  list item and append it on parent.
  ------------------------------ */
function update_list (parent, list, listener)
{
  for (let i = 0; i < list.length; i++)
  {
    let h4 = document.createElement('h4');
    h4.innerHTML = list[i];

    let li = document.createElement('li');
    li.setAttribute('class', 'w3-display-container w3-bar w3-hover-cyan');
    li.addEventListener('click', listener );
    li.appendChild(h4);
    parent.appendChild(li);
  }
}

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
  - Show previous section
  ------------------------------ */
function prev_section(id, prev, actual)
{
  toggle_visibility(id + prev, 'block');
  toggle_visibility(id + actual, 'none');
}

/* ------------------------------
  - Without params the section with id as 'running', 'logs' and 'packages'
  are hidden, and each controls button are unselected
  (with class 'w3-cyan' is consider selected)
  - With @id set on 'block' the display value to tag identified by id
  - With @button set button as selected
  - With @fun execute this function, useful to initialize the section
  ------------------------------ */
function switch_controls(id, button, fun)
{
  controls = document.getElementById('controls').children;
  for (let i = 0; i < controls.length; i++)
    controls[i].classList.remove('w3-cyan');

  // hides the section if it is currently shown
  if (id && (document.getElementById(id)).style.display == 'block')
  {
    toggle_visibility(id);
    return;
  }

  toggle_visibility('services', 'none');
  toggle_visibility('launch', 'none');
  toggle_visibility('wnodes', 'none');
  toggle_visibility('logs', 'none');
  
  // With @id
  if(id)
  {
    toggle_visibility(id, 'block');
    toggle_visibility(id + '_1', 'block');
  }

  // With @button
  if (button)
    button.classList.add('w3-cyan');

  // With @fun
  if (fun)
    fun();
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

  setTimeout( () => {
    // reset all buttons
    switch_controls();
    // clear all section
    clear_services();
    clear_launch();
    clear_wnodes();

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
