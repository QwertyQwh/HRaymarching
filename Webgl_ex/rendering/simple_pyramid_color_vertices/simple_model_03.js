/**
 * simple_model_03.js, By Wayne Brown, Spring 2016
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
 * A triangle composed of 3 vertices and a color.
 * @param vertices Array The triangle's vertices.
 * @param colors Array An array of 3 color values.
 * @constructor
  */
window.Triangle3 = function (vertices, colors) {
  var self = this;
  self.vertices = vertices;
  self.colors = colors;
}

//-------------------------------------------------------------------------
/**
 * A simple model composed of an array of triangles.
 * @param name String The name of the model.
 * @constructor
 */
window.SimpleModel2 = function (name) {
  var self = this;
  self.name = name;
  self.triangles = [];
}

//-------------------------------------------------------------------------
/**
 * Create a Simple_model of 2 triangles that forms a rectangle.
 * @return SimpleModel2
 */
window.CreatePyramid3 = function () {
  var vertices, triangle1, triangle2, triangle3, triangle4;
  var red, green, blue, purple;

  // Vertex data
  vertices = [  [ 1., 1., 0.0],
                [ 1., -1.0,  0.0],
                [ -1.0, -1.0,  0.0],
                [-1.0, 1.0,  0.0] ];

  // Colors in RGB
  red    = [1.0, 0.0, 0.0];
  green  = [0.0, 1.0, 0.0];
  blue   = [0.0, 0.0, 1.0];
  purple = [1.0, 0.0, 1.0];

  // Create 2 triangles
  triangle1 = new Triangle3([vertices[0], vertices[2], vertices[1]],
                            [blue, green, purple]);
  triangle2 = new Triangle3([vertices[0], vertices[3], vertices[2]],
                            [blue, red, green]);

  // Create a model that is composed of 2 triangles
  var model = new SimpleModel2("simple");
  model.triangles = [ triangle1, triangle2 ];

  return model;
}
