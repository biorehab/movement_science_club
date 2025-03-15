---
title: "Kinematics of a Planar Arm"
date: "2025-03-10"
layout: "post.njk"  # Must match exactly with `post.njk`
author: "Sivakumar Balasubramanian"
tags: ["blog"]
---
<style>
  svg {
    display: block;
    margin: auto;
  }

  .button-container {
    display: flex;           /* Use flexbox for layout */
    justify-content: center; /* Center-align buttons horizontally */
    align-items: center;     /* Center-align buttons vertically (if needed) */
    margin-top: 20px;        /* Add some space above the buttons */
}
</style>

<script src="https://cdnjs.cloudflare.com/ajax/libs/mathjs/11.11.1/math.min.js"></script>

<script>
    // Some utility functions
    function mapPositionsToCoordinates(positions, origin) {
        return positions.map(pos => [origin.x + pos[0],
                                     origin.y - pos[1]]);
    }

    function degreesToRadians(degreesArray) {
        return degreesArray.map(deg => deg * Math.PI / 180);
    }
    
    function radiansToDegrees(radiansArray) {
        return radiansArray.map(rad => rad * 180 / Math.PI);
    }

    function cummulativeSum(array) {
        return array.reduce((acc, curr) => {
            acc.push(acc[acc.length - 1] + curr);
            return acc;
        }, [0]).slice(1);
    }

    function polarToCartesian(centerX, centerY, radius, angleInDegrees) {
        const angleInRadians = angleInDegrees;
        return {
            x: centerX + (radius * Math.cos(angleInRadians)),
            y: centerY - (radius * Math.sin(angleInRadians))
        };
    }

    function describeArc(x, y, radius, startAngle, endAngle) {
        const start = polarToCartesian(x, y, radius, startAngle);
        const end = polarToCartesian(x, y, radius, endAngle);
        const largeArcFlag = endAngle - startAngle <= 180 ? "0" : "1";
        const sweetDir = endAngle - startAngle >= 0 ? "0" : "1";
        return `
            M ${start.x} ${start.y}
            A ${radius} ${radius} 0 ${largeArcFlag} ${sweetDir} ${end.x} ${end.y}
        `;
    }

    function polToCart(r, theta) {
        return { x: r * Math.cos(theta), y: r * Math.sin(theta) };
    }

    function cartToPol(x, y) {
        return { r: Math.hypot(x, y), theta: Math.atan2(y, x) };
    }

    // Three link arm class.
    class ThreeLinkArm {
        #jacobian = math.zeros(3, 3);

        constructor(lengths, angles) {
            this.lengths = lengths;
            this.angles = angles;
            this.#updateJacobian();
        }
    
        setAngles(newAngles) {
            this.angles = newAngles;
            this.#updateJacobian();
        }

        setLengths(newLengths) {
            this.lengths = newLengths;
            this.#updateJacobian();
        }
    
        #updateJacobian() {
            const _sterms = cummulativeSum(this.angles).map(angle => Math.sin(angle));
            const _cterms = cummulativeSum(this.angles).map(angle => Math.cos(angle));
            this.#jacobian.set([0, 0], -this.lengths[0] * _sterms[0] - this.lengths[1] * _sterms[1] - this.lengths[2] * _sterms[2]);
            this.#jacobian.set([0, 1], -this.lengths[1] * _sterms[1] - this.lengths[2] * _sterms[2]);
            this.#jacobian.set([0, 2], -this.lengths[2] * _sterms[2]);
            this.#jacobian.set([1, 0], this.lengths[0] * _cterms[0] + this.lengths[1] * _cterms[1] + this.lengths[2] * _cterms[2]);
            this.#jacobian.set([1, 1], this.lengths[1] * _cterms[1] + this.lengths[2] * _cterms[2]);
            this.#jacobian.set([1, 2], this.lengths[2] * _cterms[2]);
            this.#jacobian.set([2, 0], 1);
            this.#jacobian.set([2, 1], 1);
            this.#jacobian.set([2, 2], 1);
        }

        getJacobian() {
            return this.#jacobian;
        }

        getArmPositions() {
            const cusumtheta = cummulativeSum(this.angles);
            const x1 = 0, y1 = 0;
            const x2 = this.lengths[0] * Math.cos(cusumtheta[0]);
            const y2 = this.lengths[0] * Math.sin(cusumtheta[0]);
            const x3 = x2 + this.lengths[1] * Math.cos(cusumtheta[1]);
            const y3 = y2 + this.lengths[1] * Math.sin(cusumtheta[1]);
            const x4 = x3 + this.lengths[2] * Math.cos(cusumtheta[2]);
            const y4 = y3 + this.lengths[2] * Math.sin(cusumtheta[2]);
            return [[x1, y1], [x2, y2], [x3, y3], [x4, y4]];
        }

        forwardStatics(torque) {
            try {
                const pinvJ = math.pinv(math.transpose(this.getJacobian()));
                return math.multiply(pinvJ, math.reshape(torque, [-1, 1])).toArray().flat();
            } catch (error) {
                return null;
            }
        }
    }

    // Four link arm class
    class FourLinkArm {
        #jacobian = math.zeros(4, 4);

        constructor(lengths, angles) {
            this.lengths = lengths;
            this.angles = angles;
            // this.angles = [0.88, 0.32, 2.05, 1.25];
            this.#updateJacobian();
        }

        setAngles(newAngles) {
            this.angles = newAngles;
            this.#updateJacobian();
        }

        setLengths(newLengths) {
            this.lengths = newLengths;
            this.#updateJacobian();
        }

        #updateJacobian() {
            const _sterms = cummulativeSum(this.angles).map(angle => Math.sin(angle));
            const _cterms = cummulativeSum(this.angles).map(angle => Math.cos(angle));
            this.#jacobian.set([0, 0], -this.lengths[0] * _sterms[0] - this.lengths[1] * _sterms[1] - this.lengths[2] * _sterms[2] - this.lengths[3] * _sterms[3]);
            this.#jacobian.set([0, 1], -this.lengths[1] * _sterms[1] - this.lengths[2] * _sterms[2] - this.lengths[3] * _sterms[3]);
            this.#jacobian.set([0, 2], -this.lengths[2] * _sterms[2] - this.lengths[3] * _sterms[3]);
            this.#jacobian.set([0, 3], -this.lengths[3] * _sterms[3]);
            this.#jacobian.set([1, 0], this.lengths[0] * _cterms[0] + this.lengths[1] * _cterms[1] + this.lengths[2] * _cterms[2] + this.lengths[3] * _cterms[3]);
            this.#jacobian.set([1, 1], this.lengths[1] * _cterms[1] + this.lengths[2] * _cterms[2] + this.lengths[3] * _cterms[3]);
            this.#jacobian.set([1, 2], this.lengths[2] * _cterms[2] + this.lengths[3] * _cterms[3]);
            this.#jacobian.set([1, 3], this.lengths[3] * _cterms[3]);
            this.#jacobian.set([2, 0], 1);
            this.#jacobian.set([2, 1], 1);
            this.#jacobian.set([2, 2], 1);
            this.#jacobian.set([2, 3], 1);
        }

        getJacobian() {
            return this.#jacobian;
        }

        getArmPositions() {
            const cusumtheta = cummulativeSum(this.angles);
            const x1 = 0, y1 = 0;
            const x2 = this.lengths[0] * Math.cos(cusumtheta[0]);
            const y2 = this.lengths[0] * Math.sin(cusumtheta[0]);
            const x3 = x2 + this.lengths[1] * Math.cos(cusumtheta[1]);
            const y3 = y2 + this.lengths[1] * Math.sin(cusumtheta[1]);
            const x4 = x3 + this.lengths[2] * Math.cos(cusumtheta[2]);
            const y4 = y3 + this.lengths[2] * Math.sin(cusumtheta[2]);
            const x5 = x4 + this.lengths[3] * Math.cos(cusumtheta[3]);
            const y5 = y4 + this.lengths[3] * Math.sin(cusumtheta[3]);
            return [[x1, y1], [x2, y2], [x3, y3], [x4, y4], [x5, y5]];
        }

        getEndpointAngle() {
            return math.sum(this.angles);
        }

        forwardStatics(torque) {
            try {
                const pinvJ = math.pinv(math.transpose(this.getJacobian()));
                return math.multiply(pinvJ, math.reshape(torque, [-1, 1])).toArray().flat();
            } catch (error) {
                return null;
            }
        }
    }

    // Button callbacks.
    // Generate a random pose
    function generateRandomPose() {
        const randomAngles = [
            Math.random() * Math.PI,
            Math.random() * Math.PI * 0.75,
            Math.random() * Math.PI * 0.75,
            Math.random() * Math.PI * 0.5,
        ];
        arm4Link.setAngles(randomAngles);
        draw4LinkArm(arm4Link);
    }

    // Find other solutions (inverse kinematics)
    function findOtherIKSolutions() {
        function getArcAngles(interAngles, jPos, l) {
            // Get an angle from the minor and major arc.
            let minAngle, majAngle;
            if (interAngles[1] - interAngles[0] > Math.PI) {
                majAngle = (interAngles[1] + interAngles[0]) / 2;
                minAngle = majAngle + Math.PI;
            } else {
                minAngle = (interAngles[1] + interAngles[0]) / 2;
                majAngle = minAngle + Math.PI;
            }
            // Find positions of the points corresponding to these angles.
            const minPos = [jPos[0] + l * Math.cos(minAngle),
                            jPos[1] + l * Math.sin(minAngle)];
            const majPos = [jPos[0] + l * Math.cos(majAngle),
                            jPos[1] + l * Math.sin(majAngle)];
            // Check which one is closer to the origin.
            if (Math.hypot(minPos[0], minPos[1]) > Math.hypot(majPos[0], majPos[1])) {
                // Choose the major arc.
                if (Math.abs(interAngles[1] - interAngles[0]) < Math.PI) {
                    interAngles[0] = 2 * Math.PI + interAngles[0];
                }
            } else {
                // Choose the minor arc.
                if (Math.abs(interAngles[1] - interAngles[0]) > Math.PI) {
                    interAngles[0] = 2 * Math.PI + interAngles[0];
                }
            }
            return interAngles;
        }

        // Function to compute joint 3 position when joint 4 is not fully within
        // the workspace of the first two links.
        function getJoint3Position1(arm, j4pos) {
            // Find the intersection of the circle centered at j4Pos with radius l3 
            // and the circle centered at j1 with radius l1 + l2.
            const intersections = findCircleIntersections(0, 0, arm.lengths[0] + arm.lengths[1], j4Pos[0], j4Pos[1], arm.lengths[2]);
            // Find the angles corresponding to the intersection points.
            let interAngles = intersections.map(intpos => Math.atan2(intpos[1] - j4Pos[1], intpos[0] - j4Pos[0]));
            // Sort the interAngles
            interAngles.sort((a, b) => a - b);
            // Get the corrected arc angles.
            interAngles = getArcAngles(interAngles, j4Pos, arm4Link.lengths[2]);
            // Choose the angle of the third joint to be a number between the intersection angles.
            let j3Angle = interAngles.length > 1 ? interAngles[0] + Math.random() * (interAngles[1] - interAngles[0]) : interAngles[0];
            // Find the position on the arc.
            return {
                pos:[j4Pos[0] + arm.lengths[2] * Math.cos(j3Angle),
                     j4Pos[1] + arm.lengths[2] * Math.sin(j3Angle)],
                angle: j3Angle
            };
        }

        // Function to compute joint 3 position when joint 4 is fully within
        // the workspace of the first two links.
        function getJoint3Position2(arm, j4pos) {
            // Choose the angle of the third joint to be a number between the intersection angles.
            let j3Angle = Math.random() * Math.PI * 2;
            // Find the position on the arc.
            return {
                pos:[j4Pos[0] + arm.lengths[2] * Math.cos(j3Angle),
                     j4Pos[1] + arm.lengths[2] * Math.sin(j3Angle)],
                angle: j3Angle
            };
        }

        // Get the endpoint position
        const j4Pos = arm4Link.getArmPositions()[3];
        let _tempos = mapPositionsToCoordinates([j4Pos], arm4LinkParams.origin);
        // Relative distance of the third joint.
        const delpos = arm4Link.lengths[0] + arm4Link.lengths[1] - Math.hypot(j4Pos[0], j4Pos[1]);
        let j3 = null;
        if (delpos > arm4Link.lengths[2]) {
            j3 = getJoint3Position2(arm4Link, j4Pos);
        } else {
            j3 = getJoint3Position1(arm4Link, j4Pos);
        }
        const j3Pos = j3.pos;
        const j3Angle = j3.angle;
        _tempos = mapPositionsToCoordinates([j3Pos], arm4LinkParams.origin);

        // Find intersection of the circle centered at j3Pos with radius l2 and the circle centered at j1 with radius l1.
        let intersections2 = findCircleIntersections(0, 0, arm4Link.lengths[0],
                                                     j3Pos[0], j3Pos[1], arm4Link.lengths[1]);
        // Find the joint1 and joint2 angles corresponding to the intersection points.
        const j1Angles = intersections2.map(intpos => Math.atan2(intpos[1], intpos[0]));
        const phiAngles = intersections2.map(intpos => Math.PI + Math.atan2(intpos[1] - j3Pos[1], intpos[0] - j3Pos[0]));
        // We want the point with a positive elbow angle.
        let newAngles = [0, 0, 0, 0];
        if (j1Angles.length == 1) {
            newAngles[0] = j1Angles[0];
            newAngles[1] = phiAngles[0] - j1Angles[0];
        } else {
            if (phiAngles[0] - j1Angles[0] > 0) {
                newAngles[0] = j1Angles[0];
                newAngles[1] = phiAngles[0] - j1Angles[0];
            } else {
                newAngles[0] = j1Angles[1];
                newAngles[1] = phiAngles[1] - j1Angles[1];
            }
        }
        // Update angles 3 and 4.
        newAngles[2] = Math.PI + j3Angle - newAngles[0] - newAngles[1];
        newAngles[3] = arm4Link.getEndpointAngle() - newAngles[0] - newAngles[1] - newAngles[2];
        // Update angles and draw
        arm4Link.setAngles(newAngles);
        draw4LinkArm(arm4Link);
    }

    function findCircleIntersections(x1, y1, r1, x2, y2, r2) {
        const d = Math.hypot(x2 - x1, y2 - y1); // Distance between centers

        // No solutions if the circles do not intersect or one circle is inside the other
        if (d > r1 + r2 || d < Math.abs(r1 - r2)) {
            return null;
        }

        // Find the midpoint of the intersection line
        const a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        const h = Math.sqrt(r1 * r1 - a * a);
        
        // Base point of the perpendicular line
        const x0 = x1 + a * (x2 - x1) / d;
        const y0 = y1 + a * (y2 - y1) / d;

        // Intersection points
        const rx = -(y2 - y1) * (h / d);
        const ry = (x2 - x1) * (h / d);

        const intersection1 = [x0 + rx, y0 + ry];
        const intersection2 = [x0 - rx, y0 - ry];

        return d === r1 + r2 || d === Math.abs(r1 - r2) ? [intersection1] : [intersection1, intersection2];
    }

    // Figure 1: 3 link arm plotting function.
    function draw3LinkArm() {
        const width = 300, height = 300;
        const origin = {x: 50, y: 250};
        const viewpad = 25;

        // Create the arm object once and reuse it
        const arm = new ThreeLinkArm(
            [100, 100, 75], // Initial lengths
            degreesToRadians([30, 30, 60]) // Initial angles
        );

        const svg = d3.select("#svg-container")
                      .append("svg")
                      .attr("width", width)
                      .attr("height", height)
                      .attr("viewBox", `-${viewpad} -${viewpad} ${width + 50} ${height + 50}`)
                      .attr("preserveAspectRatio", "xMidYMid meet");

        // Plot the x and y axes
        svg.append("line")
           .attr("x1", 0).attr("y1", origin.y)
           .attr("x2", width).attr("y2", origin.y)
           .attr("stroke", "gray")
           .attr("stroke-width", 0.25);
        svg.append("line")
           .attr("x1", origin.x).attr("y1", 0)
           .attr("x2", origin.x).attr("y2", height)
           .attr("stroke", "gray")
           .attr("stroke-width", 0.25);

        // Plot the links.
        const linkpos = mapPositionsToCoordinates(arm.getArmPositions(), origin);
        for (let i = 0; i < linkpos.length - 1; i++) {
            let x1 = linkpos[i][0];
            let y1 = linkpos[i][1];
            let x2 = linkpos[i + 1][0];
            let y2 = linkpos[i + 1][1];
            // Plot the line
            svg.append("line")
                .attr("x1", linkpos[i][0])
                .attr("y1", linkpos[i][1])
                .attr("x2", linkpos[i+1][0])
                .attr("y2", linkpos[i+1][1])
                .attr("stroke", "black")
                .attr("stroke-width", 2);
            // Display the link lengths
            let midx = (x1 + x2) / 2;
            let midy = (y1 + y2) / 2;
            let shiftx = (y2 - y1) / arm.lengths[i];
            let shifty = -(x2 - x1) / arm.lengths[i];
            let shiftscalex = shiftx > 0 ? -1 : 1;
            let shiftscaley = shiftx > 0 ? -1 : 1;
            svg.append("text")
                .attr("x", midx + shiftscalex * 15 * shiftx)
                .attr("y", midy + shiftscaley * 15 * shifty)
                .attr("text-anchor", "middle")
                .attr("font-size", "14px")
                .attr("fill", "#00c")
                .text(`l${i + 1}`);
        }
        // Extend links for annotating the joint angles.
        for (let i = 1; i < linkpos.length; i++) {
            svg.append("line")
               .attr("x1", linkpos[i][0])
               .attr("y1", linkpos[i][1])
               .attr("x2", linkpos[i][0] + 0.5 * (linkpos[i][0] - linkpos[i - 1][0]))
               .attr("y2", linkpos[i][1] + 0.5 * (linkpos[i][1] - linkpos[i - 1][1]))
               .attr("stroke", "gray")
               .attr("stroke-width", "1")
               .attr("stroke-dasharray", "2,2");
        }
        // Plot the joints.
        linkpos.slice(0, -1).forEach(pos => {
            svg.append("circle")
                .attr("cx", pos[0])
                .attr("cy", pos[1])
                .attr("r", 5)
                .attr("stroke", "black")
                .attr("stroke-width", "1")
                .attr("fill", "#ffffff");
        });
        // Plot the endpoint.
        svg.append("circle")
            .attr("cx", linkpos.at(-1)[0])
            .attr("cy", linkpos.at(-1)[1])
            .attr("r", 2)
            .attr("stroke", "black")
            .attr("stroke-width", "1")
            .attr("fill", "black");
        // Draw the arc for the joint angles.
        arm.angles.map((_t, i) => {
            let startAngle = math.sum(arm.angles.slice(0, i));
            let endAngle = math.sum(arm.angles.slice(0, i + 1));
            let midAngle = (startAngle + endAngle) / 2;  // Midpoint for label
            svg.append("path")
                .attr("d", describeArc(linkpos[i][0], linkpos[i][1], 0.4 * arm.lengths[i], startAngle, endAngle))
                .attr("stroke", "black")
                .attr("fill", "none")
                .attr("stroke-width", 1.0);
            // Convert midAngle to radians
            let midAngleRad = midAngle;
            // Compute text position
            let textX = linkpos[i][0] + 0.6 * arm.lengths[i] * Math.cos(midAngleRad);
            let textY = linkpos[i][1] - 0.5 * arm.lengths[i] * Math.sin(midAngleRad);
            // Append the angle label
            svg.append("text")
                .attr("x", textX)
                .attr("y", textY)
                .attr("font-size", "14px")
                .attr("fill", "#c00")
                .attr("text-anchor", "middle")
                .text(`θ${i + 1}`);
        });
        // Display endpoint text.
        svg.append("text")
           .attr("x", linkpos.at(-1)[0] - 25)
           .attr("y", linkpos.at(-1)[1])
           .attr("font-size", "18px")
           .attr("fill", "#080")
           .attr("text-anchor", "middle")
           .text(`x, y`);
        // Plot the endpoint orientation arc and text
        svg.append("line")
           .attr("x1", linkpos.at(-1)[0])
           .attr("y1", linkpos.at(-1)[1])
           .attr("x2", linkpos.at(-1)[0] + 40)
           .attr("y2", linkpos.at(-1)[1])
           .attr("stroke", "gray")
           .attr("stroke-width", "1")
           .attr("stroke-dasharray", "2,2");
        // Angle arc
        svg.append("path")
            .attr("d", describeArc(linkpos.at(-1)[0], linkpos.at(-1)[1], 0.25 * arm.lengths.at(-1), 0, math.sum(arm.angles)))
            .attr("stroke", "black")
            .attr("fill", "none")
            .attr("stroke-width", 1.0);
        // Compute text position
        let textX = linkpos.at(-1)[0] + 0.4 * arm.lengths.at(-1) * Math.cos(math.sum(arm.angles) / 2);
        let textY = linkpos.at(-1)[1] - 0.3 * arm.lengths.at(-1) * Math.sin(math.sum(arm.angles) / 2);
        // Append the angle label
        svg.append("text")
            .attr("x", textX)
            .attr("y", textY)
            .attr("font-size", "18px")
            .attr("fill", "#080")
            .attr("text-anchor", "middle")
            .text(`ϕ`);
    }

    // Draw the four link arm.
    function draw4LinkArm(arm) {
        svg4Link.selectAll("*").remove(); // Clear previous drawing

        // Plot the x and y axes
        svg4Link.append("line")
            .attr("x1", 0).attr("y1", arm4LinkParams.origin.y)
            .attr("x2", arm4LinkParams.width).attr("y2", arm4LinkParams.origin.y)
            .attr("stroke", "gray")
            .attr("stroke-width", 0.25);
        svg4Link.append("line")
            .attr("x1", arm4LinkParams.origin.x).attr("y1", 0)
            .attr("x2", arm4LinkParams.origin.x).attr("y2", arm4LinkParams.height)
            .attr("stroke", "gray")
            .attr("stroke-width", 0.25);

        // Plot the links
        const linkpos = mapPositionsToCoordinates(arm.getArmPositions(), arm4LinkParams.origin);
        for (let i = 0; i < linkpos.length - 1; i++) {
            svg4Link.append("line")
                .attr("x1", linkpos[i][0])
                .attr("y1", linkpos[i][1])
                .attr("x2", linkpos[i + 1][0])
                .attr("y2", linkpos[i + 1][1])
                .attr("stroke", "black")
                .attr("stroke-width", 2);
        }

        // Plot the joints.
        linkpos.slice(0, -1).forEach(pos => {
            svg4Link.append("circle")
                    .attr("cx", pos[0])
                    .attr("cy", pos[1])
                    .attr("r", 3)
                    .attr("stroke", "black")
                    .attr("stroke-width", "1")
                    .attr("fill", "#ffffff");
        });
        // Plot the endpoint.
        svg4Link.append("circle")
                .attr("cx", linkpos.at(-1)[0])
                .attr("cy", linkpos.at(-1)[1])
                .attr("r", 2)
                .attr("stroke", "black")
                .attr("stroke-width", "1")
                .attr("fill", "black");

        // Display endpoint position and orientation.
        const _epstr = `x = ${arm.getArmPositions().at(-1)[0].toFixed(2)}, y = ${arm.getArmPositions().at(-1)[1].toFixed(2)}, ϕ = ${((180 / Math.PI) * math.sum(arm.angles)).toFixed(2)}`;
        svg4Link.append("text")
                .attr("x", 0)
                .attr("y", 10)
                .attr("font-size", "12px")
                .attr("fill", "#000")
                .text(_epstr);
        const jointAnglesString = radiansToDegrees(arm.angles).map((angle, i) => `θ${i + 1} = ${angle.toFixed(2)}°`).join(", ");    
        svg4Link.append("text")
                .attr("x", 0)
                .attr("y", 30)
                .attr("font-size", "12px")
                .attr("fill", "#000")
                .text(jointAnglesString);
    }

    // Automatically number items.
    function autoNumberItems() {
        let count = 1; // Start numbering from 1
        document.querySelectorAll("ol").forEach((ol) => {
            ol.setAttribute("start", count);
            count += ol.children.length;
        });
    }

    // Draw the the three link figure when the document is loaded.
    document.addEventListener("DOMContentLoaded", function() {
        draw3LinkArm();
        // Initialize the 4link SVG
        svg4Link = d3.select("#svg-4link-interactive")
                     .append("svg")
                     .attr("width", arm4LinkParams.width)
                     .attr("height", arm4LinkParams.height)
                     .attr("viewBox", `-${0} -${0} ${arm4LinkParams.width} ${arm4LinkParams.height}`)
                     .attr("preserveAspectRatio", "xMidYMid meet");
        draw4LinkArm(arm4Link);

        // Auto number items.
        autoNumberItems();
    });
    
    // Global variables.
    const arm4Link = new FourLinkArm(
        [50, 50, 50, 30], // Initial lengths
        degreesToRadians([30, 30, 30, 60])
    );
    const arm4LinkParams = {
        width:400,
        height: 400,
        origin: null
    };
    arm4LinkParams.origin = {x: arm4LinkParams.width / 2,
                             y: arm4LinkParams.height / 2};
    let svg4Link = null;
</script>

Mastering the mechanics of a planar arm is a necessary step to understanding more complex kinematic chains, such the human upper limb. In this post, we will focus on the kinematics of planar open kinematic chains composed $1$ to $n$ revolute joints.

<div id="svg-container"></div>

The above figure shows a 3 link planar arm with three revolute joints. If we think of this as the human arm, then the first joint at the origin could be thought of as the shoulder, the second joint as the elbow, and the third joint as the wrist. Let the lengths of the links are $l_1$, $l_2$, and $l_3$, and the joint angles $\theta_1$, $\theta_2$, and $\theta_3$. The position of the endpoint (black circle at the tip of the third link) $x, y$ and the orientation of the last link $\phi$ can be computed using the forward kinematics equations,

$$
\begin{split}
    x &= l_1 c_1 + l_2 c_{12} + l_3 c_{123} \\\\
    y &= l_1 s_1 + l_2 s_{12} + l_3 s_{123} \\\\
    \phi &= \theta_1 + \theta_2 + \theta_3
\end{split}
$$

where, $s_{12\cdots n} = \sin(\theta_1 + \theta_2 + \cdots + \theta_n)$ and $c_{12\cdots n} = \cos(\theta_1 + \theta_2 + \cdots + \theta_n)$.

The above equations are called the forward kinematics equations, which take us from the known joint angles to the position and orientation of the endpoint. The inverse kinematics equations, on the other hand, take us from the known position and orientation of the endpoint to the joint angles. The inverse kinematics equations are harder to solve, and in general, there may be multiple solutions or no solutions at all.

The forward kinematics equations can be generalized to $n$ link arm as the following,
$$
\begin{split}
    x &= l_1 c_1 + l_2 c_{12} + \cdots + l_n c_{123\cdots n} \\\\
    y &= l_1 s_1 + l_2 s_{12} + \cdots + l_n s_{123\cdots n} \\\\
    \phi &= \theta_1 + \theta_2 + \cdots + \theta_n
\end{split}
$$

A rigid body in a plane has $3$ degrees of freedom (DOF), $2$ for position and $1$ for orientation. A $n$ link arm has $n$ rigid bodies each with three degrees of freedom. But these rigid bodies are joined by $n$ revolute joints (one connecting link 1 to the ground and rest between the rigid bodies). Each revolute joint introduces a position constraint, i.e., $2$ constraints. Thus an $n$ link arm has $3n - 2n = n$ degrees of freedom. Thus an $n$-link arm's kinematic configuration can be completely specified by $n$ joint angles or <i>generalized coordinates</i>, i.e. $\theta_1, \theta_2 \cdots \theta_n$.

When the number of DOF of a planar arm equals $3$, there is a one-to-one mapping between the joint angles and the position and orientation of the endpoint. 

<div class="question-box">
<ol>
  <li>How can we verify this is true?</li>
</ol>
</div>

But when the number of DOF is more than $3$, there are infinitely many joint angles that will give us the same endpoint position and orientation. We say that the arm is <span class="emphasized">kinematically redundant</span> in this particular case, i.e. $n > 3$.

<div class="question-box">
<ol>
  <li>What happens when $n < 3$?</li>
  <li>When $n=3$, and we are only interested in the endpoint position. Is this arm kinematically redundant?</li>
</ol>
</div>

The following interactive simulation demonstrates this redundancy. When you press the "Generate Random Pose" button, the joint angles are randomly generated, and the arm is drawn. When you press the "Find Other Solutions" button, one of the other joint angle configurations that results in the same endpoint position and link 4 orientation is chosen and the arm is updated.
<div id="svg-4link-interactive"></div>
<div class="button-container">
    <button onclick="generateRandomPose()">Generate Random Pose</button>
    <button onclick="findOtherIKSolutions()">Find Other Solutions</button>
</div>

<!-- <h3>Differential Kinematics</h3> -->
