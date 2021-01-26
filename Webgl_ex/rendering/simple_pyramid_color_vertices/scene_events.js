/**
 * These event handlers can modify the characteristics of a scene.
 * These will be specific to a scene's models and the models' attributes.
 */

/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 C. Wayne Brown
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

"use strict";

//-------------------------------------------------------------------------
/**
 * Event handlers for
 * @param scene Object An object that knows the scene data and can render the scene.
 * @param control_id_list Array A list of control ID's .
 * @param canvas_id String The HTML id for the canvas.
 * @constructor
 */
function SimpleEvents_03(scene, control_id_list, canvas_id) {

  var self = this;

  // Private variables
  var canvas = scene.canvas;

  // Remember the current state of events
  var start_of_mouse_drag = null;
  var previous_time = Date.now();
  var animate_is_on = scene.animate_active;

  // Control the rate at which animations refresh
  var frame_rate = 33; // 33 milliseconds = 1/30 sec

  //-----------------------------------------------------------------------
  self.mouse_drag_started = function (event) {

    //console.log("started mouse drag event x,y = " + event.clientX + " " + event.clientY + "  " + event.which);
    start_of_mouse_drag = event;
    event.preventDefault();

    if (animate_is_on) {
      scene.animate_active = false;
    }
  };

  //-----------------------------------------------------------------------
  self.mouse_drag_ended = function (event) {

    //console.log("ended mouse drag event x,y = " + event.clientX + " " + event.clientY + "  " + event.which);
    start_of_mouse_drag = null;

    event.preventDefault();

    if (animate_is_on) {
      scene.animate_active = true;
      self.animate();
    }
  };

  //-----------------------------------------------------------------------
  /**
   * Process a mouse drag event.
   * @param event A jQuery event object
   */
  self.mouse_dragged = function (event) {
    var delta_x, delta_y;

    //console.log("drag event x,y = " + event.clientX + " " + event.clientY + "  " + event.which);
    if (start_of_mouse_drag) {
      delta_x = event.clientX - start_of_mouse_drag.clientX;
      delta_y = -(event.clientY - start_of_mouse_drag.clientY);
      //console.log("moved: " + delta_x + " " + delta_y);

      scene.angle_x += delta_y;
      scene.angle_y -= delta_x;
      scene.render();

      start_of_mouse_drag = event;
      event.preventDefault();
    }
  };

  //-----------------------------------------------------------------------
  self.key_event = function (event) {
    var bounds, keycode;

    bounds = canvas.getBoundingClientRect();
    // console.log("bounds = " + bounds.left + " " + bounds.right + "  " + bounds.top + "  " + bounds.bottom);
    // console.log("target = " + event.target);
    if (event.clientX >= bounds.left &&
      event.clientX <= bounds.right &&
      event.clientY >= bounds.top &&
      event.clientY <= bounds.bottom) {
      keycode = (event.keyCode ? event.keyCode : event.which);
      // console.log(keycode + " keyboard event in canvas");
    }

    return false;
  };

  //------------------------------------------------------------------------------
  self.html_control_event = function (event) {
    var control;

    control = $(event.target);
    if (control) {
      switch (control.attr('id')) {
        case "my_pause":
          if (control.is(":checked"))  {
            scene.animated = true;           
          } else {
            scene.animated = false;
          }
          scene.render();
          scene.out.displayInfo("pause is clicked");
          break;
        case "type_1" :
        case "type_2":
        case "type_3":
        case "type_4":
          scene.type = parseInt(control.val());
          scene.render();
          break;
        case "directshadow":
        case "softshadow":
        case "noshadow":
          scene.softShadow = parseInt(control.val());
          scene.out.displayInfo("shadow"+parseInt(control.val()));
          scene.render();
          break;
        case "ro_x":
          scene.ro_x = parseFloat(control.val());
          scene.render();
          break;
        case "ro_y":
          scene.ro_y = parseFloat(control.val());
          scene.render();
          break;
        case "ro_z":
          scene.ro_z = parseFloat(control.val());
          scene.render();
          break;
        case "fov":
          scene.fov =  parseFloat(control.val());
          scene.render();
          break;
      }
    }
  };

  //------------------------------------------------------------------------------
  self.createAllEventHandlers = function () {
    var j, control;
    for (j = 0; j < control_id_list.length; j += 1) {
      control = $('#' + control_id_list[j]);
      if (control) {
        if (j<8){
          control.click( self.html_control_event );          
        }
        else{
          control.change( self.html_control_event );              
        }
      }
    }
  };

  //------------------------------------------------------------------------------
  self.removeAllEventHandlers = function () {
    var j, control;
    for (j = 0; j < control_id_list.length; j += 1) {
      control = $('#' + control_id_list[j]);
      if (control) {
        if (j<8){
          control.unbind("click", self.html_control_event);        
        }
        else{
          control.unbind("change", self.html_control_event);           
        }
      }
    }
  };
  

  //------------------------------------------------------------------------------
  // Constructor code after the functions are defined.

  // Add an 'onclick' callback to each HTML control
  self.createAllEventHandlers();

  // Setup callbacks for mouse events in the canvas window.
  var id = '#' + canvas_id;
  $( id ).mousedown( self.mouse_drag_started );
  $( id ).mouseup( self.mouse_drag_ended );
  $( id ).mousemove( self.mouse_dragged );
}



