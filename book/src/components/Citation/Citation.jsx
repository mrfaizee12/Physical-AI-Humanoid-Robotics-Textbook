import React from 'react';
import clsx from 'clsx';
import styles from './Citation.module.css';

/**
 * Component for displaying IEEE format citations
 * @param {Object} props - Component properties
 * @param {string} props.id - Unique citation identifier
 * @param {string} props.authors - Authors of the citation
 * @param {string} props.title - Title of the work
 * @param {string} props.journal - Journal or conference name
 * @param {string} props.year - Year of publication
 * @param {string} [props.volume] - Volume number (if applicable)
 * @param {string} [props.number] - Number/issue (if applicable)
 * @param {string} [props.pages] - Page numbers (if applicable)
 * @param {string} [props.doi] - DOI (if applicable)
 * @param {string} [props.className] - Additional CSS classes
 */
export default function Citation({
  id,
  authors,
  title,
  journal,
  year,
  volume,
  number,
  pages,
  doi,
  className
}) {
  // Format the citation according to IEEE standards
  let citationText = `${authors}, "${title}," ${journal}`;

  if (volume) citationText += `, vol. ${volume}`;
  if (number) citationText += `, no. ${number}`;
  if (pages) citationText += `, pp. ${pages}`;
  citationText += `, ${year}.`;

  if (doi) citationText += ` doi: ${doi}.`;

  return (
    <div className={clsx(styles.citationContainer, className)} id={`cite-${id}`}>
      <sup>[{id}]</sup> {citationText}
    </div>
  );
}