/**
 * Given
 *   - a model definition as defined in learn_webgl_model_01.js, and
 *   - specific shader programs: vShader01.vert, fShader01.frag
 * Perform the following tasks:
 *   1) Build appropriate Vertex Object Buffers (VOB's)
 *   2) Create GPU VOB's for the data and copy the data into the buffers.
 *   3) Render the VOB's
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
 * Given a model description, create the buffer objects needed to render
 * the model. This is very closely tied to the shader implementations.
 * @param gl Object The WebGL state and API
 * @param program Object The shader program the will render the model.
 * @param model Simple_model The model data.
 * @param model_color The color of the model faces.
 * @param out Object Can display messages to the webpage.
 * @constructor
 */
window.SimpleModelRender_03 = function (gl, program, model, out) {

  var self = this;

  // Variables to remember so the model can be rendered.
  var number_triangles = 0;
  var triangles_vertex_buffer_id = null;
  var triangles_color_buffer_id = null;
  var then = 0.;
  var time = 0.;
  var realtime = 0.;
  // Shader variable locations
  var a_Vertex_location = null;
  var a_Color_location = null;
  var resolutionLocation = null;
  var timeLocation = null;
  var typeLocation = null;
  var softShadowLocation = null;
  var roLocation = null;
  var fovLocation = null;
  //-----------------------------------------------------------------------
  /**
   * Create a Buffer Object in the GPU's memory and upload data into it.
   * @param gl Object The WebGL state and API
   * @param data TypeArray An array of data values.
   * @returns Number a unique ID for the Buffer Object
   * @private
   */
  function _createBufferObject(gl, data) {
    // Create a buffer object
    var buffer_id;

    buffer_id = gl.createBuffer();
    if (!buffer_id) {
      out.displayError('Failed to create the buffer object for ' + model.name);
      return null;
    }

    // Make the buffer object the active buffer.
    gl.bindBuffer(gl.ARRAY_BUFFER, buffer_id);

    // Upload the data for this buffer object to the GPU.
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);

    return buffer_id;
  }

  //-----------------------------------------------------------------------
  /**
   * Using the model data, build a 1D array for the Buffer Object
   * @private
   */
  function _buildBufferObjectData() {
    var j, k, m, nv, numberVertices, triangle, vertex, all_vertices;
    var nc, all_colors, color;

    // Create a 1D array that holds all of the  for the triangles
    if (model.triangles.length > 0) {
      number_triangles = model.triangles.length;
      numberVertices = number_triangles * 3;
      all_vertices = new Float32Array(numberVertices * 3);
      all_colors = new Float32Array(numberVertices * 3);

      nv = 0;
      nc = 0;
      for (j = 0; j < model.triangles.length; j += 1) {
        triangle = model.triangles[j];

        for (k = 0; k < 3; k += 1) {
          vertex = triangle.vertices[k];
          color = triangle.colors[k];

          // Store the vertices.
          for (m = 0; m < 3; m += 1, nv += 1) {
            all_vertices[nv] = vertex[m];
          }

          // Store the colors.
          for (m = 0; m < 3; m += 1, nc += 1) {
            all_colors[nc] = color[m];
          }
        }
      }

      triangles_vertex_buffer_id = _createBufferObject(gl, all_vertices);
      triangles_color_buffer_id = _createBufferObject(gl, all_colors);
    }

    // Release the temporary vertex array so the memory can be reclaimed.
    all_vertices = null;
    all_colors = null;
  }

  //-----------------------------------------------------------------------
  /**
   * Get the location of the shader variables in the shader program.
   * @private
   */
  function _getLocationOfShaderVariables() {
    // Get the location of the shader variables

    a_Vertex_location    = gl.getAttribLocation(program, 'a_Vertex');
    a_Color_location     = gl.getAttribLocation(program, 'a_Color');

    resolutionLocation = gl.getUniformLocation(program, "iResolution");
    timeLocation = gl.getUniformLocation(program, "iTime");
    typeLocation = gl.getUniformLocation(program, "iType");    
    softShadowLocation = gl.getUniformLocation(program, "iSoftShadow");   
    roLocation = gl.getUniformLocation(program, "iRo");
    fovLocation = gl.getUniformLocation(program, "iFov");
  }

  //-----------------------------------------------------------------------
  // These one-time tasks set up the rendering of the models.
  _buildBufferObjectData();
  _getLocationOfShaderVariables();

  //-----------------------------------------------------------------------
  /**
   * Delete the Buffer Objects associated with this model.
   * @param gl Object The WebGL state and API.
   */
  self.delete = function (gl) {
    if (number_triangles > 0) {
      gl.deleteBuffer(triangles_vertex_buffer_id);
    }
  };

  //-----------------------------------------------------------------------
  /**
   * Render the model.
   * @param gl Object The WebGL state and API.
   * @param transform 4x4Matrix The transformation to apply to the model vertices.
   */
  self.type = 1;
  self.softShadow = 2;
  self.animated = true;
  self.ro_x = 0.;
  self.ro_y = 0.;
  self.ro_z = 0.;
  self.fov= 0.25; 
  self.render = function (now,type,softShadow,animated,ro_x,ro_y,ro_z,fov) {
    now *= 0.001;  // convert to seconds
    const elapsedTime = Math.min(now - then, 0.1);
    realtime += elapsedTime;
    self.animated = animated
    if (self.animated){
      time  += elapsedTime;
    }
    then = now;
    // Set the transform for all the triangle vertices
    gl.uniform2f(resolutionLocation, gl.canvas.width, gl.canvas.height);
    gl.uniform1f(timeLocation, time);
    self.type = type;
    gl.uniform1i(typeLocation,type);
    self.softShadow = softShadow;
    gl.uniform1i(softShadowLocation,softShadow);
    self.ro_x = ro_x;
    self.ro_y = ro_y;
    self.ro_z = ro_z;
    gl.uniform3f(roLocation,ro_x,ro_y,ro_z);
    self.fov= fov;
    gl.uniform1f(fovLocation,fov);   

    // Activate the model's vertex Buffer Object
    gl.bindBuffer(gl.ARRAY_BUFFER, triangles_vertex_buffer_id);

    // Bind the vertices Buffer Object to the 'a_Vertex' shader variable
    gl.vertexAttribPointer(a_Vertex_location, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(a_Vertex_location);

    // Activate the model's color Buffer Object
    gl.bindBuffer(gl.ARRAY_BUFFER, triangles_color_buffer_id);

    // Bind the color Buffer Object to the 'a_Color' shader variable
    gl.vertexAttribPointer(a_Color_location, 3, gl.FLOAT, false, 0, 0);
    gl.enableVertexAttribArray(a_Color_location);

    
    // Draw all of the triangles
    gl.drawArrays(gl.TRIANGLES, 0, number_triangles * 3);
    requestAnimationFrame(function (lambda){self.render(lambda,self.type,self.softShadow,self.animated,self.ro_x,self.ro_y,self.ro_z,self.fov)});
  };

};
