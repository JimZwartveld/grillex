import * as THREE from 'three';

export interface SectionProfile {
  shape: THREE.Shape;
  width: number;
  height: number;
}

/**
 * Create an I-beam cross-section profile
 */
export function createIBeamProfile(
  h: number,   // Total height
  b: number,   // Flange width
  tw: number,  // Web thickness
  tf: number   // Flange thickness
): SectionProfile {
  const shape = new THREE.Shape();

  const halfH = h / 2;
  const halfB = b / 2;
  const halfTw = tw / 2;

  // Draw I-beam cross-section (centered at origin)
  // Start at bottom-left of bottom flange
  shape.moveTo(-halfB, -halfH);
  shape.lineTo(halfB, -halfH);         // Bottom flange bottom
  shape.lineTo(halfB, -halfH + tf);    // Bottom flange right
  shape.lineTo(halfTw, -halfH + tf);   // Web right bottom
  shape.lineTo(halfTw, halfH - tf);    // Web right top
  shape.lineTo(halfB, halfH - tf);     // Top flange right
  shape.lineTo(halfB, halfH);          // Top flange top right
  shape.lineTo(-halfB, halfH);         // Top flange top left
  shape.lineTo(-halfB, halfH - tf);    // Top flange left
  shape.lineTo(-halfTw, halfH - tf);   // Web left top
  shape.lineTo(-halfTw, -halfH + tf);  // Web left bottom
  shape.lineTo(-halfB, -halfH + tf);   // Bottom flange left
  shape.closePath();

  return { shape, width: b, height: h };
}

/**
 * Create a hollow box (RHS/SHS) cross-section profile
 */
export function createBoxProfile(
  h: number,   // Height
  b: number,   // Width
  t: number    // Wall thickness
): SectionProfile {
  const shape = new THREE.Shape();

  // Outer rectangle
  shape.moveTo(-b / 2, -h / 2);
  shape.lineTo(b / 2, -h / 2);
  shape.lineTo(b / 2, h / 2);
  shape.lineTo(-b / 2, h / 2);
  shape.closePath();

  // Inner rectangle (hole)
  const inner = new THREE.Path();
  const innerB = b / 2 - t;
  const innerH = h / 2 - t;
  inner.moveTo(-innerB, -innerH);
  inner.lineTo(-innerB, innerH);
  inner.lineTo(innerB, innerH);
  inner.lineTo(innerB, -innerH);
  inner.closePath();
  shape.holes.push(inner);

  return { shape, width: b, height: h };
}

/**
 * Create a channel (C-section) cross-section profile
 */
export function createChannelProfile(
  h: number,   // Total height
  b: number,   // Flange width
  tw: number,  // Web thickness
  tf: number   // Flange thickness
): SectionProfile {
  const shape = new THREE.Shape();

  const halfH = h / 2;

  // Draw C-section (web on left side)
  shape.moveTo(0, -halfH);
  shape.lineTo(b, -halfH);              // Bottom flange bottom
  shape.lineTo(b, -halfH + tf);         // Bottom flange tip
  shape.lineTo(tw, -halfH + tf);        // Web right bottom
  shape.lineTo(tw, halfH - tf);         // Web right top
  shape.lineTo(b, halfH - tf);          // Top flange tip
  shape.lineTo(b, halfH);               // Top flange top
  shape.lineTo(0, halfH);               // Web top
  shape.closePath();

  return { shape, width: b, height: h };
}

/**
 * Create an angle (L-section) cross-section profile
 */
export function createAngleProfile(
  h: number,   // Vertical leg length
  b: number,   // Horizontal leg length
  t: number    // Thickness
): SectionProfile {
  const shape = new THREE.Shape();

  shape.moveTo(0, 0);
  shape.lineTo(b, 0);
  shape.lineTo(b, t);
  shape.lineTo(t, t);
  shape.lineTo(t, h);
  shape.lineTo(0, h);
  shape.closePath();

  return { shape, width: b, height: h };
}

/**
 * Create a circular pipe cross-section profile
 */
export function createPipeProfile(
  d: number,   // Outer diameter
  t: number    // Wall thickness
): SectionProfile {
  const shape = new THREE.Shape();

  // Outer circle
  const outerRadius = d / 2;
  shape.absarc(0, 0, outerRadius, 0, Math.PI * 2, false);

  // Inner circle (hole)
  const innerRadius = outerRadius - t;
  if (innerRadius > 0) {
    const inner = new THREE.Path();
    inner.absarc(0, 0, innerRadius, 0, Math.PI * 2, true);
    shape.holes.push(inner);
  }

  return { shape, width: d, height: d };
}

/**
 * Create a solid rectangular cross-section profile
 */
export function createRectProfile(
  h: number,   // Height
  b: number    // Width
): SectionProfile {
  const shape = new THREE.Shape();

  shape.moveTo(-b / 2, -h / 2);
  shape.lineTo(b / 2, -h / 2);
  shape.lineTo(b / 2, h / 2);
  shape.lineTo(-b / 2, h / 2);
  shape.closePath();

  return { shape, width: b, height: h };
}

/**
 * Parse section name and create appropriate profile
 * Returns a simple rectangular profile if section type is unknown
 */
export function createProfileFromSection(
  sectionName: string,
  sectionProperties: { A: number; Iy: number; Iz: number; J: number }
): SectionProfile {
  // Try to parse section name to determine type and dimensions
  const name = sectionName.toUpperCase();

  // IPE sections (I-beam)
  if (name.startsWith('IPE')) {
    const heightStr = name.replace('IPE', '');
    const h = parseFloat(heightStr) / 1000; // Convert mm to m
    // Approximate IPE dimensions based on height
    const b = h * 0.5;
    const tw = h * 0.05;
    const tf = h * 0.08;
    return createIBeamProfile(h, b, tw, tf);
  }

  // HEA/HEB/HEM sections (wide flange I-beam)
  if (name.startsWith('HE')) {
    const heightStr = name.replace(/HE[ABM]/, '');
    const h = parseFloat(heightStr) / 1000;
    const b = h * 1.0; // HE sections have roughly equal width
    const tw = h * 0.06;
    const tf = h * 0.10;
    return createIBeamProfile(h, b, tw, tf);
  }

  // UPN/UPE channels
  if (name.startsWith('UPN') || name.startsWith('UPE')) {
    const heightStr = name.replace(/UP[NE]/, '');
    const h = parseFloat(heightStr) / 1000;
    const b = h * 0.4;
    const tw = h * 0.05;
    const tf = h * 0.08;
    return createChannelProfile(h, b, tw, tf);
  }

  // RHS/SHS (rectangular/square hollow sections)
  if (name.includes('X')) {
    const parts = name.split('X').map((s) => parseFloat(s));
    if (parts.length >= 2) {
      const h = parts[0] / 1000;
      const b = parts[1] / 1000;
      const t = parts.length >= 3 ? parts[2] / 1000 : Math.min(h, b) * 0.1;
      return createBoxProfile(h, b, t);
    }
  }

  // CHS (circular hollow section)
  if (name.startsWith('CHS') || name.includes('D')) {
    const match = name.match(/(\d+)/);
    if (match) {
      const d = parseFloat(match[1]) / 1000;
      const t = d * 0.1;
      return createPipeProfile(d, t);
    }
  }

  // L-sections (angles)
  if (name.startsWith('L')) {
    const match = name.match(/L(\d+)X(\d+)/);
    if (match) {
      const h = parseFloat(match[1]) / 1000;
      const b = parseFloat(match[2]) / 1000;
      const t = Math.min(h, b) * 0.1;
      return createAngleProfile(h, b, t);
    }
  }

  // Default: estimate dimensions from section properties
  const A = sectionProperties.A;
  const Iy = sectionProperties.Iy;

  // Rough estimation: for a rectangle h*b = A and h^3*b/12 = Iy
  // So h/b = 12*Iy/A^2 (very rough approximation)
  let h = Math.sqrt(12 * Iy / Math.sqrt(A)) || 0.3;
  let b = A / h || 0.15;

  // Clamp to reasonable values
  h = Math.max(0.1, Math.min(h, 1.0));
  b = Math.max(0.05, Math.min(b, 0.5));

  return createRectProfile(h, b);
}
