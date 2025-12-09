import React from 'react';
import clsx from 'clsx';
import styles from './Diagram.module.css';

/**
 * Component for displaying diagrams in the textbook
 * @param {Object} props - Component properties
 * @param {string} props.src - Source path of the diagram
 * @param {string} props.alt - Alt text for accessibility
 * @param {string} [props.caption] - Optional caption for the diagram
 * @param {string} [props.className] - Additional CSS classes
 */
export default function Diagram({ src, alt, caption, className }) {
  return (
    <div className={clsx(styles.diagramContainer, className)}>
      <img src={src} alt={alt} className={styles.diagramImage} />
      {caption && <div className={styles.diagramCaption}>{caption}</div>}
    </div>
  );
}